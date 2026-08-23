//! OPA (operational amplifier) demo on OPA2.
//!
//! OPA2 is used because its non-inverting input (PA7) and its output pad
//! (PA5) are also ADC channels 7 and 5, so the same signal can be measured
//! before the amplifier, inside the chip, and on the output pad.
//!
//! Wiring:
//!   - jumper PA5 to PA6, so the output feeds the inverting input directly
//!     and the amplifier is a unity-gain buffer;
//!   - drive PA7 with something between 0 V and VCCA (a pot, or just leave it
//!     floating and watch the three readings track each other).
//!
//! The three ADC readings should agree within a few LSB. Note the amplifier
//! output range is 0.1 V to VCCA - 0.2 V, so it clips near the rails.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use py32_hal::adc::{Adc, SampleTime};
use py32_hal::opa::{Ch2, Opa};
use py32_hal::pac;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = py32_hal::init(Default::default());

    info!("Hello World!");

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES71_5);

    let mut vin = p.PA7;
    let mut vout = p.PA5;

    let opa = Opa::new(p.OPA);
    let mut opa2 = opa.enable::<Ch2>(vin.reborrow(), p.PA6);
    opa2.enable_output(vout.reborrow());

    // The temperature sensor shares the internal-channel block with the OPA
    // outputs; on DIE072 it is ADC_IN23, not ADC_IN16 (which is OPA3). At room
    // temperature the raw reading must land close to the factory calibration
    // value taken at 30 degC — if it does not, the channel number is wrong.
    let ts_cal_30c = pac::CONFIGBYTES.ts_cal_30c().read().tscal();
    let mut temperature = adc.enable_temperature();
    let temp = adc.blocking_read(&mut temperature);
    info!("temperature: {} (ts_cal_30c: {})", temp, ts_cal_30c);
    if temp.abs_diff(ts_cal_30c) > 400 {
        defmt::panic!("temperature sensor reading is nowhere near TS_CAL_30C");
    }

    loop {
        let before = adc.blocking_read(&mut vin);
        let internal = adc.blocking_read(&mut opa2);
        let pad = adc.blocking_read(&mut vout);
        info!(
            "PA7 in: {}  OPA2 internal: {}  PA5 pad: {}",
            before, internal, pad
        );
        Timer::after_millis(200).await;
    }
}
