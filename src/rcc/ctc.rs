//! CTC support for the PLL48M output.

// The following code is modified from embassy-stm32
// https://github.com/embassy-rs/embassy/tree/main/embassy-stm32
// Special thanks to the Embassy Project and its contributors for their work!

use crate::pac::CTC;
use crate::pac::ctc::vals::{Refpsc, Refsel};
use crate::rcc;
use crate::time::Hertz;

/// PLL48M speed.
pub const PLL48M_FREQ: Hertz = Hertz(48_000_000);

/// Configuration for the CTC (PLL48M trimmer).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CtcConfig {
    /// Enable CTC sync from USB Start Of Frame (SOF) events.
    ///
    /// Required if the PLL48M clock is going to be used as USB clock.
    ///
    /// Other use cases of the CTC are not supported yet.
    pub sync_from_usb: bool,
}

impl CtcConfig {
    pub const fn new() -> Self {
        Self {
            sync_from_usb: false,
        }
    }
}

impl Default for CtcConfig {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) fn init_ctc(config: CtcConfig) -> Hertz {
    rcc::enable_and_reset::<crate::peripherals::CTC>();

    if config.sync_from_usb {
        // These are the correct settings for standard USB operation:
        // one SOF pulse per millisecond means the expected count between
        // two sync pulses is 48 MHz / 1 kHz = 48000. If other settings are
        // needed there will need to be additional config options for the CTC.
        //
        // CTL1 must be programmed before enabling the counter: while CNTEN
        // is set, CTL1 cannot be modified.
        CTC.ctl1().write(|w| {
            w.set_refsel(Refsel::USBDSOF);
            w.set_refpsc(Refpsc::DIV1);
            w.set_cklim(0x22);
            w.set_rlvalue(48_000);
        });

        // Clear any stale status flags.
        CTC.intc().write(|w| {
            w.set_ckokic(true);
            w.set_ckwarnic(true);
            w.set_erric(true);
            w.set_erefic(true);
        });

        // modify (not write): preserves the reset TRIMVALUE of 64.
        CTC.ctl0().modify(|w| {
            w.set_autotrim(true);
            w.set_cnten(true);
        });
    }

    PLL48M_FREQ
}

/// Returns the current hardware trimming step (`CTL0.TRIMVALUE`).
///
/// One step corresponds to roughly ±0.25 % of the PLL48M frequency. While
/// automatic trimming is active, hardware adjusts this value itself.
pub fn trim_value() -> u8 {
    CTC.ctl0().read().trimvalue()
}

/// Returns the counter value captured on the last reference pulse
/// (`SR.REFCAP`).
///
/// With USB SOF sync this is the number of PLL48M cycles counted during the
/// last millisecond, nominally 48000.
pub fn ref_capture() -> u16 {
    CTC.sr().read().refcap()
}
