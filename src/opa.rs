//! Operational amplifier (OPA).
//!
//! The PY32 OPA block holds three independent op-amps behind a single RCC
//! gate. There is nothing to configure beyond enabling an amplifier and
//! optionally routing its output to the dedicated pad: gain is set with
//! external components, and the output always reaches the ADC and the
//! comparators internally.
//!
//! | Op-amp | + (VP) | − (VN) | OUT pad | ADC channel | COMP |
//! |---|---|---|---|---|---|
//! | [`Ch1`] | PA9 | PA10 | PA8 | 21 | COMP1_INP0 |
//! | [`Ch2`] | PA7 | PA6 | PA5 | 22 | COMP2_INP9 |
//! | [`Ch3`] | PB13 | PB12 | PB14 | 16 | COMP3_INP4 |
//!
//! Pins and ADC channels above are the PY32F040/F071/F072 mapping. PY32F031
//! shares the same die entry but wires OPA2 to PB3/PA15/PB4 and reports the
//! outputs on ADC channels 13/12, so the pin traits are wrong there; it also
//! has no OPA3.
//!
//! The following code is modified from embassy-stm32
//! <https://github.com/embassy-rs/embassy/tree/main/embassy-stm32>
//! Special thanks to the Embassy Project and its contributors for their work!

#![macro_use]

use core::marker::PhantomData;

use embassy_hal_internal::Peri;

use crate::pac;
use crate::peripherals::{ADC1, OPA};
use crate::rcc;

trait SealedChannel {
    /// Index into the `CR0.OPAOEN1[]` / `CR1.EN[]` field arrays.
    const INDEX: usize;
    /// ADC channel the amplifier output is wired to internally.
    const ADC_CHANNEL: u8;
}

/// Op-amp marker trait.
#[allow(private_bounds)]
pub trait Channel: SealedChannel {}

macro_rules! impl_channel {
    ($name:ident, $index:expr, $adc_channel:expr) => {
        #[doc = concat!("OPA", stringify!($index), " marker type.")]
        pub enum $name {}
        impl SealedChannel for $name {
            const INDEX: usize = $index;
            const ADC_CHANNEL: u8 = $adc_channel;
        }
        impl Channel for $name {}
    };
}

impl_channel!(Ch1, 0, 21);
impl_channel!(Ch2, 1, 22);
impl_channel!(Ch3, 2, 16);

/// A pin that can be an op-amp's non-inverting input.
pub trait VpPin<C: Channel>: crate::gpio::Pin {}
/// A pin that can be an op-amp's inverting input.
pub trait VnPin<C: Channel>: crate::gpio::Pin {}
/// A pin that can carry an op-amp's output.
pub trait VoutPin<C: Channel>: crate::gpio::Pin {}

#[allow(unused_macros)]
macro_rules! impl_opa_pin {
    ($inst:ident, $pin:ident, $trait:ident, $ch:ident) => {
        impl crate::opa::$trait<crate::opa::$ch> for crate::peripherals::$pin {}
    };
}

/// OPA driver.
pub struct Opa<'d> {
    _peri: Peri<'d, OPA>,
}

impl<'d> Opa<'d> {
    /// Instantiates the OPA peripheral.
    pub fn new(peripheral: Peri<'d, OPA>) -> Self {
        rcc::enable_and_reset::<OPA>();
        Self { _peri: peripheral }
    }

    /// Enables one op-amp on its dedicated input pins.
    ///
    /// Gain is set by whatever feedback network is wired between the output
    /// and the inverting input; with the output tied straight back to it the
    /// amplifier is a unity-gain buffer.
    ///
    /// The output is available internally to the ADC (read the returned
    /// [`OpaChannel`] as an ADC channel) and to the comparators. Use
    /// [`OpaChannel::enable_output`] to also drive the dedicated pad.
    pub fn enable<C: Channel>(
        &self,
        vp: Peri<'_, impl VpPin<C>>,
        vn: Peri<'_, impl VnPin<C>>,
    ) -> OpaChannel<'_, C> {
        vp.set_as_analog();
        vn.set_as_analog();

        // The three op-amps share one register, so read-modify-write must not
        // be interrupted by another channel being enabled or dropped.
        critical_section::with(|_| {
            pac::OPA.cr1().modify(|w| w.set_en(C::INDEX, true));
        });

        OpaChannel {
            _opa: PhantomData,
            _ch: PhantomData,
        }
    }
}

impl Drop for Opa<'_> {
    fn drop(&mut self) {
        rcc::disable::<OPA>();
    }
}

/// A single enabled op-amp.
///
/// The amplifier is disabled when this is dropped. Can be used directly as an
/// ADC channel to read the output without leaving the chip.
pub struct OpaChannel<'d, C: Channel> {
    _opa: PhantomData<&'d Opa<'d>>,
    _ch: PhantomData<C>,
}

impl<'d, C: Channel> OpaChannel<'d, C> {
    /// Routes the amplifier output to its dedicated pad.
    pub fn enable_output(&mut self, vout: Peri<'_, impl VoutPin<C>>) {
        vout.set_as_analog();
        critical_section::with(|_| {
            pac::OPA.cr0().modify(|w| w.set_opaoen1(C::INDEX, true));
        });
    }

    /// Stops driving the dedicated output pad. The output stays available
    /// internally to the ADC and the comparators.
    pub fn disable_output(&mut self) {
        critical_section::with(|_| {
            pac::OPA.cr0().modify(|w| w.set_opaoen1(C::INDEX, false));
        });
    }
}

impl<C: Channel> Drop for OpaChannel<'_, C> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            pac::OPA.cr0().modify(|w| w.set_opaoen1(C::INDEX, false));
            pac::OPA.cr1().modify(|w| w.set_en(C::INDEX, false));
        });
    }
}

impl<C: Channel> crate::adc::AdcChannel<ADC1> for OpaChannel<'_, C> {}
impl<C: Channel> crate::adc::SealedAdcChannel<ADC1> for OpaChannel<'_, C> {
    fn channel(&self) -> u8 {
        C::ADC_CHANNEL
    }
}
