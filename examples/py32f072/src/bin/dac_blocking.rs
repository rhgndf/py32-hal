#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use py32_hal::dac::{Dac, u12r};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = py32_hal::init(Default::default());
    let mut dac = Dac::new_blocking(p.DAC, p.PA4, p.PA5);

    // PA4 is approximately VDDA / 4 and PA5 is approximately 3 * VDDA / 4.
    dac.set((u12r::new(1024), u12r::new(3072)));
    info!("DAC outputs enabled: PA4=1024/4095, PA5=3072/4095");

    loop {
        Timer::after_secs(1).await;
    }
}
