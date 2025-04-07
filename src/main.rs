#![no_std]
#![no_main]
use core::cell::RefCell;
use cortex_m::asm::nop;
use defmt::{assert_eq, println, unwrap};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, Error, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::{
    CriticalSectionRawMutex, NoopRawMutex, ThreadModeRawMutex,
};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c as async_i2c;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

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
async fn gyro_worker(
    fxas: fxas2100::FXAS2100<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>>,
    mut i2c: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
) {
    let mut data = [0u8; 6];
    let mut collect = false;

    loop {
        if collect {
            let g_data = GYRO_DATA.lock().await;
            match i2c.write_read(FXAS2100_ADDRESS, &*g_data, &mut data).await {
                Ok(_) => {
                    println!("data collected")
                }
                Err(_) => {
                    println!("Encountered an error");
                }
            }
        }
        if fxas.collect_signal.signaled() {
            match fxas.collect_signal.try_take() {
                Some(v) => {
                    collect = v;
                    if !collect {
                        fxas.collect_signal.wait().await;
                    }
                }
                None => nop(),
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut data = [0u8; 1];
    let mut odr_data = [0u8; 3];

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
    // let _ = i2c.write_read(FXAS2100_ADDRESS, &[CTRL_REG1], &mut odr_data);
    let i2c_bus = Mutex::new(i2c);
    let i2c_bus = I2C2_BUS.init(i2c_bus);
    let i2c2_device1 = I2cDevice::new(i2c_bus);
    let i2c2_device2 = I2cDevice::new(i2c_bus);
    let i2c2_device3 = I2cDevice::new(i2c_bus);
    let i2c2_device4 = I2cDevice::new(i2c_bus);

    let mut fxas = fxas2100::FXAS2100::new(i2c2_device4, FXAS2100_ADDRESS, gyro_signal);
    let who_am_i = fxas.read_register(fxas2100::registers::WHO_AM_I).await;
    let current_state = fxas.set_active().await;
    assert_eq!(0xD7, who_am_i);
    println!("Made it past the assert");
    let _ = spawner.spawn(read_i2c_whoami(i2c2_device1));
    let _ = spawner.spawn(read_ctrl_reg1(i2c2_device2));
    let _ = spawner.spawn(gyro_worker(fxas, i2c2_device3));
    println!("tick");

    loop {
        Timer::after_millis(100).await;
        println!("tick");
        println!("Gyro Data: {}", *GYRO_DATA.lock().await);
    }
}
