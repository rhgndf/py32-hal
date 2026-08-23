#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use py32_hal::lptim::{Config, Lptim, Mode, Prescaler};
use py32_hal::rcc::{self, LsConfig};
use py32_hal::{bind_interrupts, lptim};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    TIM6_LPTIM1_DAC => lptim::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Enable LSI and use it as the LPTIM1 kernel clock.
    let mut config: py32_hal::Config = Default::default();
    config.rcc.ls = Some(LsConfig { lsi: true, lse: false });
    config.rcc.mux.lptim1sel = rcc::mux::Lptim1sel::LSI;
    let p = py32_hal::init(config);
    info!("Hello World!");

    let mut lptim = Lptim::new(
        p.LPTIM1,
        Irqs,
        Config {
            prescaler: Prescaler::DIV1,
            mode: Mode::Continuous,
            ..Default::default()
        },
    );

    info!(
        "kernel clock: {} Hz",
        rcc::frequency::<py32_hal::peripherals::LPTIM1>()
    );

    // 1 s match period from the 32.768 kHz LSI clock.
    lptim.set_autoreload(32767);
    lptim.start();

    loop {
        lptim.wait_match().await;
        info!("match, counter: {}", lptim.counter());
    }
}
