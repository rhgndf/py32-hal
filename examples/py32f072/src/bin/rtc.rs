#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use py32_hal::rtc::{Rtc, RtcClockSource, RtcConfig, RtcInterrupt};
use py32_hal::{bind_interrupts, rtc};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RTC => rtc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = py32_hal::init(Default::default());
    info!("Hello World!");

    // RTC ticking at 1 Hz from the 32.768 kHz LSI (prescaler 32767).
    let mut rtc = Rtc::new(
        p.RTC,
        Irqs,
        RtcConfig {
            clock_source: RtcClockSource::Lsi,
            prescaler: 32767,
        },
    );

    rtc.set_counter(0);
    rtc.enable_interrupt(RtcInterrupt::Second);
    rtc.set_alarm(5);
    rtc.enable_interrupt(RtcInterrupt::Alarm);

    loop {
        rtc.wait_second().await;
        info!("tick: {}", rtc.now());
        if rtc.interrupt_flag(RtcInterrupt::Alarm) {
            rtc.clear_interrupt_flag(RtcInterrupt::Alarm);
            info!("ALARM!");
            rtc.set_alarm(rtc.now() + 5);
        }
    }
}
