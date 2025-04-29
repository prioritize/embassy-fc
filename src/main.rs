#![no_std]
#![no_main]
use defmt::{assert_eq, println};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c as async_i2c;
use fxas2100::registers::Registers as FXASRegisters;
use fxas2100::DEFAULT_ADDRESS as gyro_address;
use fxas2100::{State, FXAS2100};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

type AsyncFXAS = FXAS2100<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>>;
type AsyncI2CDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>;

static I2C2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> =
    StaticCell::new();

bind_interrupts!(
    struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

static _ODR_SHARED: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);
static _GYRO_DATA: Mutex<ThreadModeRawMutex, [u8; 6]> = Mutex::new([0u8; 6]);
// static GYRO_COLLECT_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static GYRO_COLLECT_SC: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
static GYRO_DATA_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, [u8; 6], 10>> =
    StaticCell::new();
static FXAS_CELL: StaticCell<Mutex<CriticalSectionRawMutex, AsyncFXAS>> = StaticCell::new();
static GYRO_BUFFER: StaticCell<Mutex<CriticalSectionRawMutex, [u8; 192]>> = StaticCell::new();

// // This function accepts the bitmask in binary "e.g. - 0b00001111" and the existing value, and will
// // toggle the bits in the mask off in the value and return that value
// const fn toggle_off(mask: u8, value: u8) -> u8 {
//     !mask & value
// }
// // This function accepts the bitmask in binary "e.g. - 0b00001111" and the existing value, and will
// // toggle the bits in the mask on in the value and return that value
// const fn toggle_on(mask: u8, value: u8) -> u8 {
//     mask | value
// }
fn combine(msb: u8, lsb: u8) -> u16 {
    (msb as u16) << 8 | lsb as u16
}
#[embassy_executor::task]
async fn read_i2c_whoami(mut i2c: AsyncI2CDevice) {
    let mut data = [0u8; 1];
    loop {
        Timer::after_secs(1).await;
        match i2c
            .write_read(gyro_address, &[FXASRegisters::WhoAmI.to_u8()], &mut data)
            .await
        {
            Ok(_) => {
                println!("{}", data[0]);
            }
            Err(_) => {
                println!("Encountered an error");
            }
        }
        assert_eq!(0xD7, data[0]);
    }
}

#[embassy_executor::task]
async fn read_ctrl_reg1(mut i2c: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>) {
    let mut data = [0u8; 6];
    match i2c
        .write_read(gyro_address, &[FXASRegisters::CtrlReg1.to_u8()], &mut data)
        .await
    {
        Ok(_) => {
            println!("CTRL_REG1: {}", data[0]);
            println!("CTRL_REG1: {}", data[0] & 0b00001110);
        }
        Err(_) => {
            println!("Encountered an error");
        }
    }
}

#[embassy_executor::task]
async fn gyro_state_handler(
    gyro: &'static Mutex<CriticalSectionRawMutex, AsyncFXAS>,
    gyro_state_signal: Signal<CriticalSectionRawMutex, State>,
) {
    loop {
        gyro_state_signal.wait().await;
        let current_state = gyro
            .lock()
            .await
            .read_register(FXASRegisters::CtrlReg1.to_u8())
            .await;
        let current_state = current_state | 0b00000011;
        let _state = match current_state {
            0 => State::Standby,
            1 => State::Ready,
            2 => State::Active,
            3 => State::Active,
            _ => State::Error,
        };
        match gyro_state_signal.try_take() {
            Some(s) => match s {
                State::Active => {
                    gyro.lock().await.set_active().await;
                }
                State::Ready => {
                    gyro.lock().await.set_ready().await;
                }
                State::Standby => {
                    gyro.lock().await.set_standby().await;
                }
                State::Error => println!("Got an error state"),
            },
            None => continue,
        }
    }
}
#[embassy_executor::task]
async fn gyro_producer(
    gyro: &'static Mutex<CriticalSectionRawMutex, AsyncFXAS>,
    output_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10>,
) {
    let timeout = 1_000_000 / gyro.lock().await.data_rate.to_u16() as u64;
    loop {
        Timer::after_micros(timeout).await;
        let mut gyro = gyro.lock().await;
        if gyro.state == State::Active {
            let data = gyro.get_gyro_data().await;
            let _ = output_channel.try_send(data);
        }
    }
}
#[embassy_executor::task]
async fn gyro_producer_fifo(
    gyro: &'static Mutex<CriticalSectionRawMutex, AsyncFXAS>,
    output_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10>,
    buffer: &'static Mutex<CriticalSectionRawMutex, [u8; 192]>,
) {
    let timeout = (1_000_000 / gyro.lock().await.data_rate.to_u16() as u64) * 24;
    loop {
        Timer::after_micros(timeout).await;
        let mut gyro = gyro.lock().await;
        if gyro.state == State::Active {
            let data = gyro.get_gyro_data_buffer(*buffer.lock().await).await;
        }
    }
}

#[embassy_executor::task]
async fn gyro_consumer(input_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10>) {
    loop {
        let data = input_channel.receive().await;
        let x = combine(data[1], data[0]);
        let y = combine(data[3], data[2]);
        let z = combine(data[5], data[4]);
        println!(
            "Data received in gyro_consumer: x: {}, y: {}, z: {}",
            x, y, z
        );
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let i2c = I2c::new(
        p.I2C2,
        p.PF1,
        p.PF0,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH2,
        Hertz(100_000),
        Default::default(),
    );
    let _gyro_signal: &'static mut Signal<CriticalSectionRawMutex, bool> =
        GYRO_COLLECT_SC.init(Signal::new());
    let gyro_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10> =
        GYRO_DATA_CHANNEL.init(Channel::new());
    let gyro_buffer: &'static Mutex<CriticalSectionRawMutex, [u8; 192]> =
        GYRO_BUFFER.init(Mutex::new([0u8; 192]));
    // let _ = i2c.write_read(FXAS2100_ADDRESS, &[CTRL_REG1], &mut odr_data);
    let i2c_bus = Mutex::new(i2c);
    let i2c_bus = I2C2_BUS.init(i2c_bus);
    let i2c2_device1 = I2cDevice::new(i2c_bus);
    let i2c2_device2 = I2cDevice::new(i2c_bus);
    let _i2c2_device3 = I2cDevice::new(i2c_bus);
    let i2c2_device4 = I2cDevice::new(i2c_bus);

    let fxas_interior = fxas2100::FXAS2100::new(i2c2_device4, gyro_address).await;
    let fxas = FXAS_CELL.init(Mutex::new(fxas_interior));
    let who_am_i = fxas
        .get_mut()
        .read_register(FXASRegisters::WhoAmI.to_u8())
        .await;
    //let current_state = fxas.set_active().await;
    assert_eq!(0xD7, who_am_i);
    println!("Made it past the assert");
    let _ = spawner.spawn(read_i2c_whoami(i2c2_device1));
    let _ = spawner.spawn(read_ctrl_reg1(i2c2_device2));

    let _ = spawner.spawn(gyro_producer_fifo(fxas, gyro_channel, gyro_buffer));
    // let _ = spawner.spawn(gyro_producer(fxas, gyro_channel));
    // let _ = spawner.spawn(gyro_consumer(gyro_channel));

    let mut value = true;
    Timer::after_secs(1).await;

    let _ = fxas
        .lock()
        .await
        .set_fifo_mode(fxas2100::fifo::Mode::Circular)
        .await;

    loop {
        Timer::after_millis(500).await;
        if value {
            fxas.lock().await.set_active().await;
            value = !value;
        } else {
            fxas.lock().await.set_ready().await;
            value = !value;
        }
    }
}
