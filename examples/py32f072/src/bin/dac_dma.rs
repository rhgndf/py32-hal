#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use py32_hal::dac::{DacChannel, Trigger, u12r};
use py32_hal::time::Hertz;
use py32_hal::timer::low_level::{MasterMode, Timer};
use {defmt_rtt as _, panic_probe as _};

// One period of a 32-sample triangle lookup table. At a 32 kHz trigger rate the output
// frequency is 1 kHz.
static WAVEFORM: [u12r; 32] = [
    u12r::new(0),
    u12r::new(256),
    u12r::new(512),
    u12r::new(768),
    u12r::new(1024),
    u12r::new(1280),
    u12r::new(1536),
    u12r::new(1792),
    u12r::new(2048),
    u12r::new(2304),
    u12r::new(2560),
    u12r::new(2816),
    u12r::new(3072),
    u12r::new(3328),
    u12r::new(3584),
    u12r::new(3840),
    u12r::new(4095),
    u12r::new(3840),
    u12r::new(3584),
    u12r::new(3328),
    u12r::new(3072),
    u12r::new(2816),
    u12r::new(2560),
    u12r::new(2304),
    u12r::new(2048),
    u12r::new(1792),
    u12r::new(1536),
    u12r::new(1280),
    u12r::new(1024),
    u12r::new(768),
    u12r::new(512),
    u12r::new(256),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = py32_hal::init(Default::default());

    let timer = Timer::new(p.TIM6);
    timer.set_frequency(Hertz::khz(32));
    timer.set_master_mode(MasterMode::UPDATE);

    let mut dac = DacChannel::new_triggered(p.DAC, p.DMA1_CH1, Trigger::Tim6, p.PA4);

    info!("starting 1 kHz DAC DMA waveform on PA4");
    timer.start();
    dac.write_circular(&WAVEFORM).await;
}
