#![no_std]
#![no_main]

use assign_resources::assign_resources;
use defmt::{panic, *};
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{Ipv6Address, Ipv6Cidr, Stack, StackResources, StaticConfigV6};
use embassy_net_adin1110::{Device, ADIN1110};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::rng::{self, Rng};
use embassy_stm32::spi::{Config as SPI_Config, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{bind_interrupts, exti, peripherals, usb, Config};
use embassy_time::Delay;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::Vec;
use panic_probe as _;
use rand_core::RngCore;
use static_cell::StaticCell;

assign_resources! {
    usb: UsbResources {
        usb_otg_fs: USB_OTG_FS,
        pa11: PA11,
        pa12: PA12,
    }
    adin_spe: SpeResources {
        exti8: EXTI8,
        gpdma1_ch12: GPDMA1_CH12,
        gpdma1_ch13: GPDMA1_CH13,
        pa0: PA0,
        pa15: PA15,
        pb3: PB3,
        pb4: PB4,
        pb5: PB5,
        pb8: PB8,
        ph1: PH1,
        spi3: SPI3,
    }
}

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

pub type SpeSpi = Spi<'static, Async>;
pub type SpeSpiCs = ExclusiveDevice<SpeSpi, Output<'static>, Delay>;
pub type SpeInt = exti::ExtiInput<'static>;
pub type SpeRst = Output<'static>;
pub type Adin1110T = ADIN1110<SpeSpiCs>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.msi = Some(MSIRange::RANGE_48MHZ);
        config.rcc.pll1 = Some(Pll {
            source: PllSource::MSIS, // 16 MHz
            prediv: PllPreDiv::DIV3,
            mul: PllMul::MUL10,
            divp: Some(PllDiv::DIV2), // 80 MHz
            divq: Some(PllDiv::DIV2), // 80 MHz
            divr: Some(PllDiv::DIV1), // 160 MHz
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.voltage_range = VoltageScale::RANGE1;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // needed for USB
        config.rcc.mux.iclksel = mux::Iclksel::HSI48; // USB uses ICLK
    }

    let p = embassy_stm32::init(config);
    let r = split_resources!(p);

    spawner.spawn(usb_serial_task(spawner, r.usb)).unwrap();

    let mut rng = Rng::new(p.RNG, Irqs);
    let seed = rng.next_u64();
    spawner
        .spawn(adin_spe_task(spawner, r.adin_spe, seed))
        .unwrap();
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

#[embassy_executor::task]
async fn usb_serial_task(_spawner: Spawner, r: UsbResources) {
    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = false;
    let driver = Driver::new_fs(
        r.usb_otg_fs,
        Irqs,
        r.pa12,
        r.pa11,
        &mut ep_out_buffer,
        config,
    );

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Sofar");
    config.product = Some("Bristlemouth mote");
    config.serial_number = Some("hermit");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();
    let usb_fut = usb.run();

    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    join(usb_fut, echo_fut).await;
}

#[embassy_executor::task]
async fn adin_spe_task(_spawner: Spawner, r: SpeResources, seed: u64) {
    const MAC: [u8; 6] = [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff];
    const IP_ADDRESS: Ipv6Cidr = Ipv6Cidr::new(
        Ipv6Address([
            0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0xc0, 0xff, 0xee, 0xee, 0xf0, 0xca, 0xcc, 0x1a,
        ]),
        64,
    );
    static STATE: StaticCell<embassy_net_adin1110::State<8, 8>> = StaticCell::new();
    let state = STATE.init(embassy_net_adin1110::State::<8, 8>::new());

    let spe_spi_cs_n = Output::new(r.pa15, Level::High, Speed::High);
    let spe_spi_sclk = r.pb3;
    let spe_spi_miso = r.pb4;
    let spe_spi_mosi = r.pb5;

    let mut spi_config = SPI_Config::default();
    spi_config.frequency = Hertz(20_000_000);
    let spe_spi: SpeSpi = Spi::new(
        r.spi3,
        spe_spi_sclk,
        spe_spi_mosi,
        spe_spi_miso,
        r.gpdma1_ch13,
        r.gpdma1_ch12,
        spi_config,
    );
    let spe_spi = SpeSpiCs::new(spe_spi, spe_spi_cs_n, Delay).unwrap();
    let spe_int = exti::ExtiInput::new(r.pb8, r.exti8, Pull::None);
    let spe_reset_n = Output::new(r.pa0, Level::Low, Speed::Low);
    let _spe_pwr_en = Output::new(r.ph1, Level::High, Speed::Low);

    // FIXME: Panicking in here unwrapping AdinError::SPI_CRC when reading PHYID
    // rx_buf in read_reg is consistently [0, 0, 0, 1, 0]
    let (device, runner) =
        embassy_net_adin1110::new(MAC, state, spe_spi, spe_int, spe_reset_n, true, false).await;
    let eth_fut = runner.run();

    let ip_cfg = embassy_net::Config::ipv6_static(StaticConfigV6 {
        address: IP_ADDRESS,
        gateway: None,
        dns_servers: Vec::new(),
    });

    static STACK: StaticCell<Stack<Device<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        device,
        ip_cfg,
        RESOURCES.init(StackResources::<2>::new()),
        seed,
    ));
    let net_fut = stack.run();

    join(eth_fut, net_fut).await;
}
