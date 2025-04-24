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
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c as async_i2c;
use fxas2100::registers::Registers as FXASRegisters;
use fxas2100::{State, FXAS2100};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

type AsyncFXAS = FXAS2100<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>>;
type AsyncI2CDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>;

const FXAS2100_ADDRESS: u8 = 0x21;
const FXAS2100_DATA: u8 = 0x01;
const WHOAMI: u8 = 0x0C;
const CTRL_REG1: u8 = 0x13;
const EXPECTED_NAME: u8 = 0xD7;

static I2C2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> =
    StaticCell::new();

bind_interrupts!(
    struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

static ODR_SHARED: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);
static GYRO_DATA: Mutex<ThreadModeRawMutex, [u8; 6]> = Mutex::new([0u8; 6]);
// static GYRO_COLLECT_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static GYRO_COLLECT_SC: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
static GYRO_DATA_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, [u8; 6], 10>> =
    StaticCell::new();
static FXAS_CELL: StaticCell<Mutex<CriticalSectionRawMutex, AsyncFXAS>> = StaticCell::new();

// This function accepts the bitmask in binary "e.g. - 0b00001111" and the existing value, and will
// toggle the bits in the mask off in the value and return that value
const fn toggle_off(mask: u8, value: u8) -> u8 {
    !mask & value
}
// This function accepts the bitmask in binary "e.g. - 0b00001111" and the existing value, and will
// toggle the bits in the mask on in the value and return that value
const fn toggle_on(mask: u8, value: u8) -> u8 {
    mask | value
}
#[embassy_executor::task]
async fn read_i2c_whoami(
    mut i2c: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
) {
    let mut data = [0u8; 1];
    loop {
        Timer::after_secs(1).await;
        match i2c.write_read(FXAS2100_ADDRESS, &[WHOAMI], &mut data).await {
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
        .write_read(FXAS2100_ADDRESS, &[CTRL_REG1], &mut data)
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
            .read_register(FXASRegisters::CTRL_REG1.to_u8())
            .await;
        let current_state = current_state | 0b00000011;
        let state = match current_state {
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
async fn gyro_worker(
    gyro: &'static Mutex<CriticalSectionRawMutex, AsyncFXAS>,
    output_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10>,
) {
    loop {
        Timer::after_millis(1).await;
        let mut gyro = gyro.lock().await;
        if gyro.state == State::Active {
            let data = gyro.collect_gyro_data().await;
            let _ = output_channel.try_send(data);
            println!("{}", data);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut data = [0u8; 1];
    let mut odr_data = [0u8; 3];
    let mut x_gyro_data = 0u16;
    let mut y_gyro_data = 0u16;
    let mut z_gyro_data = 0u16;

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
    let gyro_signal: &'static mut Signal<CriticalSectionRawMutex, bool> =
        GYRO_COLLECT_SC.init(Signal::new());
    let gyro_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10> =
        GYRO_DATA_CHANNEL.init(Channel::new());
    // let _ = i2c.write_read(FXAS2100_ADDRESS, &[CTRL_REG1], &mut odr_data);
    let i2c_bus = Mutex::new(i2c);
    let i2c_bus = I2C2_BUS.init(i2c_bus);
    let i2c2_device1 = I2cDevice::new(i2c_bus);
    let i2c2_device2 = I2cDevice::new(i2c_bus);
    let i2c2_device3 = I2cDevice::new(i2c_bus);
    let i2c2_device4 = I2cDevice::new(i2c_bus);

    let fxas_interior = fxas2100::FXAS2100::new(i2c2_device4, FXAS2100_ADDRESS).await;
    let mut fxas = FXAS_CELL.init(Mutex::new(fxas_interior));
    let who_am_i = fxas
        .get_mut()
        .read_register(FXASRegisters::WHO_AM_I.to_u8())
        .await;
    //let current_state = fxas.set_active().await;
    assert_eq!(0xD7, who_am_i);
    println!("Made it past the assert");
    let _ = spawner.spawn(read_i2c_whoami(i2c2_device1));
    let _ = spawner.spawn(read_ctrl_reg1(i2c2_device2));
    let _ = spawner.spawn(gyro_worker(fxas, gyro_channel));

    let mut value = true;
    Timer::after_secs(1).await;

    fxas.lock().await.enable_self_test().await;
    fxas.lock().await.set_active().await;

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
