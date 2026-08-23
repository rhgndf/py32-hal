#![no_std]
#![no_main]

use core::fmt::Write;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::Timer;
use heapless::String;
use py32_hal::bind_interrupts;
use py32_hal::rcc::{CtcConfig, HsiFs, Pll, PllMul, PllSource, Sysclk};
use py32_hal::usb::{Driver, InterruptHandler};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_usb::class::cdc_acm::{CdcAcmClass, State};

bind_interrupts!(struct Irqs {
    USB => InterruptHandler<py32_hal::peripherals::USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // PY32 USB uses PLL as the clock source and can only run at 48Mhz.
    // The CTC trims this same PLL48M output against USB SOF.
    let config = py32_hal::Config {
        rcc: py32_hal::rcc::Config {
            hsi: Some(HsiFs::HSI_16MHZ),
            pll: Some(Pll {
                src: PllSource::HSI,
                mul: PllMul::MUL3,
            }),
            sys: Sysclk::PLL,
            ctc: Some(CtcConfig {
                sync_from_usb: true,
            }),
            ..Default::default()
        },
        ..Default::default()
    };
    let p = py32_hal::init(config);

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);

    // Create embassy-usb Config
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("CTC trim monitor");
        config.serial_number = Some("12345678");
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        )
    };

    // Create classes on the builder.
    let mut class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    // Build the builder.
    let mut usb = builder.build();

    info!("CTC started: trimming PLL48M against USB SOF");

    // Run the USB device and report the CTC status once per second.
    join(usb.run(), async {
        loop {
            Timer::after_secs(1).await;

            let trim = py32_hal::rcc::trim_value();
            let refcap = py32_hal::rcc::ref_capture();

            let mut msg: String<64> = String::new();
            unwrap!(core::write!(msg, "trim={} refcap={}\r\n", trim, refcap));

            trace!("ctc: {:?}", msg.as_str());
            let _ = class.write_packet(msg.as_bytes()).await;
        }
    })
    .await;
}
