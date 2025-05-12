#![no_std]
#![no_main]

use defmt::{assert_eq, info, println, trace, unwrap};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpMetadata, UdpSocket};
use embassy_net::{Ipv4Address, Ipv4Cidr, StackResources};
use embassy_stm32::eth::{Ethernet, GenericPhy, PacketQueue};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::ETH;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, eth, peripherals};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c as async_i2c;
use fxas2100::registers::Registers as FXASRegisters;
use fxas2100::DEFAULT_ADDRESS as gyro_address;
use fxas2100::{State, FXAS2100};
use heapless::Vec;
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
    ETH => eth::InterruptHandler;

});
type EthernetDevice = Ethernet<'static, ETH, GenericPhy>;

static _ODR_SHARED: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);
static _GYRO_DATA: Mutex<ThreadModeRawMutex, [u8; 6]> = Mutex::new([0u8; 6]);
// static GYRO_COLLECT_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static GYRO_COLLECT_SC: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
static GYRO_DATA_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, [u8; 6], 32>> =
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
            None => {
                println!("Nothing in the try_take");
                continue;
            }
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
        if gyro.get_state() == &State::Active {
            let data = gyro.get_gyro_data().await;
            let _ = output_channel.try_send(data);
        }
    }
}
#[embassy_executor::task]
async fn gyro_producer_fifo(
    gyro: &'static Mutex<CriticalSectionRawMutex, AsyncFXAS>,
    output_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 32>,
    buffer: &'static Mutex<CriticalSectionRawMutex, [u8; 192]>,
) {
    let timeout = (1_000_000 / gyro.lock().await.data_rate.to_u16() as u64) * 20;
    loop {
        Timer::after_micros(timeout).await;
        let mut gyro = gyro.lock().await;
        if gyro.get_state() == &State::Active {
            let size = gyro.get_gyro_data_buffer(*buffer.lock().await).await as usize;
            let mut temp_buffer = [0u8; 192];
            {
                let locked_buffer = buffer.lock().await;
                temp_buffer[0..size].copy_from_slice(&locked_buffer[0..size]);
            }
            for i in (0..size / 6).step_by(6) {
                let _ = output_channel.try_send([
                    temp_buffer[i],
                    temp_buffer[i + 1],
                    temp_buffer[i + 2],
                    temp_buffer[i + 3],
                    temp_buffer[i + 4],
                    temp_buffer[i + 5],
                ]);
            }
        }
    }
}

#[embassy_executor::task]
async fn gyro_consumer(input_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 32>) {
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
#[embassy_executor::task]
async fn gyro_temp_producer(gyro: &'static Mutex<CriticalSectionRawMutex, AsyncFXAS>) {
    loop {
        Timer::after_secs(1).await;
        let temp = gyro.lock().await.get_temp().await;
        if let Ok(t) = temp {
            info!("Temperature: {}", t)
        }
    }
}
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, EthernetDevice>) -> ! {
    runner.run().await
}
#[embassy_executor::task]
async fn gyro_socket_task(
    socket: &'static mut UdpSocket<'static>,
    in_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 32>,
) {
    let remote_endpoint = (Ipv4Address::new(192, 168, 1, 29), 8000);
    let _ = socket.bind(remote_endpoint);
    loop {
        Timer::after_secs(1).await;
        socket
            .send_to(b"Hello, world", remote_endpoint)
            .await
            .expect("Buffer sent");
    }
    //
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
        Hertz(400_000),
        Default::default(),
    );
    let _gyro_signal: &'static mut Signal<CriticalSectionRawMutex, bool> =
        GYRO_COLLECT_SC.init(Signal::new());
    let gyro_channel: &'static Channel<CriticalSectionRawMutex, [u8; 6], 32> =
        GYRO_DATA_CHANNEL.init(Channel::new());
    let gyro_buffer: &'static Mutex<CriticalSectionRawMutex, [u8; 192]> =
        GYRO_BUFFER.init(Mutex::new([0u8; 192]));
    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let device = eth::Ethernet::new(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA2,
        p.PC1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PB13,
        p.PG11,
        GenericPhy::new_auto(),
        mac_addr,
    );
    let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 1, 99), 24),
        dns_servers: Vec::new(),
        gateway: Some(Ipv4Address::new(192, 168, 0, 1)),
    });

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
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) =
        embassy_net::new(device, config, RESOURCES.init(StackResources::new()), 10);
    //let current_state = fxas.set_active().await;
    assert_eq!(0xD7, who_am_i);
    println!("Made it past the assert");
    // let _ = spawner.spawn(read_i2c_whoami(i2c2_device1));
    let _ = spawner.spawn(read_ctrl_reg1(i2c2_device2));

    let _ = spawner.spawn(gyro_producer_fifo(fxas, gyro_channel, gyro_buffer));
    // let _ = spawner.spawn(gyro_producer(fxas, gyro_channel));
    let _ = spawner.spawn(gyro_consumer(gyro_channel));
    let _ = spawner.spawn(gyro_temp_producer(fxas));

    let mut value = true;
    Timer::after_secs(1).await;

    let _ = fxas
        .lock()
        .await
        .set_fifo_mode(fxas2100::fifo::Mode::Circular)
        .await;
    let _ = fxas.lock().await.set_active().await;

    unwrap!(spawner.spawn(net_task(runner)));
    stack.wait_config_up().await;
    // Then we can use it!
    static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
    static RX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    static TX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

    let rx_meta = RX_META.init([PacketMetadata::EMPTY; 16]);
    let tx_meta = TX_META.init([PacketMetadata::EMPTY; 16]);
    let rx_buffer = RX_BUFFER.init([0u8; 1024]);
    let tx_buffer = TX_BUFFER.init([0u8; 1024]);

    let remote_endpoint = (Ipv4Address::new(192, 168, 1, 29), 8000);
    let mut socket = UdpSocket::new(stack, rx_meta, rx_buffer, tx_meta, tx_buffer);
    static UDP_SOCKET: StaticCell<UdpSocket> = StaticCell::new();
    let udp_socket = UDP_SOCKET.init(socket);
    // socket.bind(remote_endpoint).expect("Socket Bind Error");
    // trace!("Is the socket ready?: {}", socket.may_send());
    unwrap!(spawner.spawn(gyro_socket_task(udp_socket, gyro_channel)));
    loop {
        Timer::after_millis(500).await;
        info!("tick");
        // socket
        //     .send_to(b"Hello, world", remote_endpoint)
        //     .await
        //     .expect("Buffer sent");
        Timer::after_secs(1).await;
        // if value {
        //     fxas.lock().await.set_active().await;
        //     value = !value;
        // } else {
        //     fxas.lock().await.set_ready().await;
        //     value = !value;
        // }
    }
}
