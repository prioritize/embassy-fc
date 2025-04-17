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
use fxas2100::commands::GyroCommands;
use fxas2100::FXAS2100;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
type AsyncFXAS =
    fxas2100::FXAS2100<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>>;
type GyroCommandChannel = Channel<CriticalSectionRawMutex, GyroCommands, 10>;
type GyroOutputChannel = Channel<CriticalSectionRawMutex, u8, 10>;

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
static GYRO_COMMAND_CHANNEL: StaticCell<GyroCommandChannel> = StaticCell::new();
static GYRO_OUTPUT_CHANNEL: StaticCell<GyroOutputChannel> = StaticCell::new();
static FXAS_CELL: StaticCell<AsyncFXAS> = StaticCell::new();

#[embassy_executor::task]
async fn gyro_controller(
    fxas: &'static mut AsyncFXAS,
    command_channel: &'static GyroCommandChannel,
) {
    fxas.controller_task(command_channel).await
}
#[embassy_executor::task]
async fn gyro_producer(channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10>) {}

#[embassy_executor::task]
async fn gyro_consumer(
    fxas: &'static AsyncFXAS,
    gyro_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 10>,
) {
    let mut data = [0u8; 6];
    let mut collect = false;
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
    let gyro_command_channel: &'static GyroCommandChannel =
        GYRO_COMMAND_CHANNEL.init(Channel::new());
    let gyro_output_channel: &'static GyroOutputChannel = GYRO_OUTPUT_CHANNEL.init(Channel::new());
    // let _ = i2c.write_read(FXAS2100_ADDRESS, &[CTRL_REG1], &mut odr_data);
    let i2c_bus = Mutex::new(i2c);
    let i2c_bus = I2C2_BUS.init(i2c_bus);
    let i2c2_device1 = I2cDevice::new(i2c_bus);
    let i2c2_device2 = I2cDevice::new(i2c_bus);
    let i2c2_device3 = I2cDevice::new(i2c_bus);
    let i2c2_device4 = I2cDevice::new(i2c_bus);

    let fxas_device = fxas2100::FXAS2100::new(i2c2_device4, FXAS2100_ADDRESS, gyro_signal);
    let fxas: &'static mut AsyncFXAS = FXAS_CELL.init(fxas_device);
    let who_am_i = fxas.read_register(fxas2100::registers::WHO_AM_I).await;
    //let current_state = fxas.set_active().await;
    assert_eq!(0xD7, who_am_i);
    println!("Made it past the assert");
    let _ = spawner.spawn(gyro_controller(fxas, gyro_command_channel));

    loop {
        Timer::after_secs(1);
        gyro_command_channel.send(GyroCommands::CheckWhoAmI).await;
    }
}
