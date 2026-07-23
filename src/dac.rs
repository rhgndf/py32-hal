//! Digital-to-analog converter (DAC).
//!
//! This driver follows the channel and word-type API used by `embassy-stm32`'s DAC driver,
//! adapted to the PY32 DMA request multiplexer and interrupt model.

#![macro_use]

use core::marker::PhantomData;
use core::slice;
use core::sync::atomic::{AtomicU8, Ordering};

#[cfg(dma)]
use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{Peri, PeripheralType};

#[cfg(dma)]
use crate::dma::{
    ChannelAndRequest, TransferOptions,
    word::{self as dma_word, WordSize},
};
#[cfg(dma)]
use crate::mode::Async;
use crate::mode::{Blocking, Mode as PeriMode};
use crate::pac::dac::Dac as Regs;
use crate::pac::dac::{regs, vals};
use crate::peripherals;
use crate::rcc::{self, RccInfo, RccPeripheral, SealedRccPeripheral};

/// A DAC trigger source.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Trigger {
    /// Timer 6 trigger output.
    Tim6,
    /// Timer 3 trigger output.
    Tim3,
    /// Timer 7 trigger output.
    Tim7,
    /// Timer 15 trigger output.
    Tim15,
    /// Timer 2 trigger output.
    Tim2,
    /// External interrupt line 9.
    Exti9,
    /// Software trigger, issued by [`DacChannel::trigger`].
    Software,
}

impl Trigger {
    fn tsel(self) -> vals::Tsel {
        match self {
            Self::Tim6 => vals::Tsel::TIM6_TRGO,
            Self::Tim3 => vals::Tsel::TIM3_TRGO,
            Self::Tim7 => vals::Tsel::TIM7_TRGO,
            Self::Tim15 => vals::Tsel::TIM15_TRGO,
            Self::Tim2 => vals::Tsel::TIM2_TRGO,
            Self::Exti9 => vals::Tsel::EXTI9,
            Self::Software => vals::Tsel::SOFTWARE,
        }
    }

    fn drives_dma(self) -> bool {
        self != Self::Software
    }
}

/// Channel 1 marker type.
pub enum Ch1 {}

/// Channel 2 marker type.
pub enum Ch2 {}

trait SealedChannel {
    const INDEX: usize;
}

/// DAC channel marker trait.
#[allow(private_bounds)]
pub trait Channel: SealedChannel {}

impl SealedChannel for Ch1 {
    const INDEX: usize = 0;
}

impl SealedChannel for Ch2 {
    const INDEX: usize = 1;
}

impl Channel for Ch1 {}
impl Channel for Ch2 {}

/// A pin that can carry a DAC channel's analog output.
pub trait DacPin<T: Instance, C: Channel>: crate::gpio::Pin {}

#[allow(unused_macros)]
macro_rules! impl_dac_pin {
    ($inst:ident, $pin:ident, 1u8) => {
        impl crate::dac::DacPin<crate::peripherals::$inst, crate::dac::Ch1>
            for crate::peripherals::$pin
        {
        }
    };
    ($inst:ident, $pin:ident, 2u8) => {
        impl crate::dac::DacPin<crate::peripherals::$inst, crate::dac::Ch2>
            for crate::peripherals::$pin
        {
        }
    };
}

#[cfg(dma)]
dma_trait!(Dma, Instance, Channel);

/// A right-aligned 12-bit DAC sample.
///
/// Only the low 12 bits are converted.
#[allow(non_camel_case_types)]
#[repr(transparent)]
#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct u12r(pub u16);

impl u12r {
    /// Construct a right-aligned sample, masking it to 12 bits.
    pub const fn new(value: u16) -> Self {
        Self(value & 0x0fff)
    }
}

/// A left-aligned 12-bit DAC sample.
///
/// The sample is stored in the same representation written to the holding register: bits 15:4
/// contain the value and bits 3:0 are zero. Use [`u12l::new`] to convert a normal 12-bit value.
#[allow(non_camel_case_types)]
#[repr(transparent)]
#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct u12l(pub u16);

impl u12l {
    /// Construct a left-aligned sample from a normal 12-bit value.
    pub const fn new(value: u16) -> Self {
        Self((value & 0x0fff) << 4)
    }
}

trait SealedWord: Sized {
    #[cfg(dma)]
    type DmaWord: dma_word::Word;

    #[cfg(dma)]
    fn dma_buf(data: &[Self]) -> &[Self::DmaWord];
    #[cfg(dma)]
    fn dma_ptr(regs: Regs, index: usize) -> *mut Self::DmaWord;
    fn set_value(regs: Regs, index: usize, value: Self);
    fn set_values(regs: Regs, values: (Self, Self));
}

/// A sample representation supported by the DAC holding registers.
#[allow(private_bounds)]
pub trait Word: SealedWord {}

impl<T: SealedWord> Word for T {}

impl SealedWord for u8 {
    #[cfg(dma)]
    type DmaWord = u8;

    #[cfg(dma)]
    fn dma_buf(data: &[Self]) -> &[Self::DmaWord] {
        data
    }

    #[cfg(dma)]
    fn dma_ptr(regs: Regs, index: usize) -> *mut Self::DmaWord {
        regs.dhr8r(index).as_ptr().cast()
    }

    fn set_value(regs: Regs, index: usize, value: Self) {
        regs.dhr8r(index).write(|w| w.set_dhr(value));
    }

    fn set_values(regs: Regs, values: (Self, Self)) {
        regs.dhr8rd().write(|w| {
            w.set_dhr(0, values.0);
            w.set_dhr(1, values.1);
        });
    }
}

impl SealedWord for u12r {
    #[cfg(dma)]
    type DmaWord = u16;

    #[cfg(dma)]
    fn dma_buf(data: &[Self]) -> &[Self::DmaWord] {
        // SAFETY: u12r is repr(transparent) over u16 and therefore has identical layout.
        unsafe { slice::from_raw_parts(data.as_ptr().cast(), data.len()) }
    }

    #[cfg(dma)]
    fn dma_ptr(regs: Regs, index: usize) -> *mut Self::DmaWord {
        regs.dhr12r(index).as_ptr().cast()
    }

    fn set_value(regs: Regs, index: usize, value: Self) {
        regs.dhr12r(index).write(|w| w.set_dhr(value.0));
    }

    fn set_values(regs: Regs, values: (Self, Self)) {
        regs.dhr12rd().write(|w| {
            w.set_dhr(0, values.0.0);
            w.set_dhr(1, values.1.0);
        });
    }
}

impl SealedWord for u12l {
    #[cfg(dma)]
    type DmaWord = u16;

    #[cfg(dma)]
    fn dma_buf(data: &[Self]) -> &[Self::DmaWord] {
        // SAFETY: u12l is repr(transparent) over u16 and therefore has identical layout.
        unsafe { slice::from_raw_parts(data.as_ptr().cast(), data.len()) }
    }

    #[cfg(dma)]
    fn dma_ptr(regs: Regs, index: usize) -> *mut Self::DmaWord {
        regs.dhr12l(index).as_ptr().cast()
    }

    fn set_value(regs: Regs, index: usize, value: Self) {
        regs.dhr12l(index)
            .write_value(regs::Dhr12l(u32::from(value.0 & 0xfff0)));
    }

    fn set_values(regs: Regs, values: (Self, Self)) {
        let ch1 = u32::from(values.0.0 & 0xfff0);
        let ch2 = u32::from(values.1.0 & 0xfff0) << 16;
        regs.dhr12ld().write_value(regs::Dhr12ld(ch1 | ch2));
    }
}

struct State {
    owners: AtomicU8,
}

impl State {
    const fn new() -> Self {
        Self {
            owners: AtomicU8::new(0),
        }
    }

    fn acquire(&self, count: u8) {
        critical_section::with(|_| {
            let owners = self.owners.load(Ordering::Relaxed);
            assert_eq!(owners, 0, "DAC peripheral is already owned");
            self.owners.store(count, Ordering::Relaxed);
        });
    }

    fn release(&self) -> bool {
        critical_section::with(|_| {
            let owners = self.owners.load(Ordering::Relaxed);
            debug_assert!(owners > 0);
            let remaining = owners - 1;
            self.owners.store(remaining, Ordering::Relaxed);
            remaining == 0
        })
    }
}

struct Info {
    regs: Regs,
    rcc: RccInfo,
}

trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

/// DAC peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + RccPeripheral + 'static {}

foreach_peripheral!(
    (dac, $inst:ident) => {
        impl crate::dac::SealedInstance for peripherals::$inst {
            fn info() -> &'static Info {
                static INFO: Info = Info {
                    regs: unsafe { Regs::from_ptr(crate::pac::$inst.as_ptr()) },
                    rcc: crate::peripherals::$inst::RCC_INFO,
                };
                &INFO
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl crate::dac::Instance for peripherals::$inst {}
    };
);

/// Driver for one DAC channel.
///
/// Use [`Dac`] when both output channels are required.
pub struct DacChannel<'d, M: PeriMode> {
    info: &'static Info,
    state: &'static State,
    index: usize,
    trigger: Option<Trigger>,
    #[cfg(dma)]
    dma: Option<ChannelAndRequest<'d>>,
    _mode: PhantomData<&'d mut M>,
}

impl<'d> DacChannel<'d, Blocking> {
    /// Create a blocking DAC channel with triggering disabled and the output buffer enabled.
    pub fn new_blocking<T: Instance, C: Channel>(
        _peri: Peri<'d, T>,
        pin: Peri<'d, impl DacPin<T, C>>,
    ) -> Self {
        pin.set_as_analog();
        rcc::enable_and_reset::<T>();
        T::state().acquire(1);
        Self::new_inner::<T, C>(None)
    }
}

#[cfg(dma)]
impl<'d> DacChannel<'d, Async> {
    /// Create an async-capable DAC channel with triggering disabled.
    ///
    /// Call [`DacChannel::set_trigger`] with an external trigger before using DMA.
    pub fn new<T: Instance, C: Channel, D: Dma<T, C>>(
        _peri: Peri<'d, T>,
        dma: Peri<'d, D>,
        pin: Peri<'d, impl DacPin<T, C>>,
    ) -> Self {
        pin.set_as_analog();
        rcc::enable_and_reset::<T>();
        T::state().acquire(1);
        Self::new_inner::<T, C>(new_dma!(dma))
    }

    /// Create an async DAC channel paced by an external trigger.
    pub fn new_triggered<T: Instance, C: Channel, D: Dma<T, C>>(
        _peri: Peri<'d, T>,
        dma: Peri<'d, D>,
        trigger: Trigger,
        pin: Peri<'d, impl DacPin<T, C>>,
    ) -> Self {
        assert!(
            trigger.drives_dma(),
            "software triggers cannot drive DAC DMA"
        );
        pin.set_as_analog();
        rcc::enable_and_reset::<T>();
        T::state().acquire(1);
        let mut channel = Self::new_inner::<T, C>(new_dma!(dma));
        channel.set_trigger(Some(trigger));
        channel
    }

    /// Write a finite sample buffer using DMA.
    ///
    /// The channel must use an external trigger. Buffers must contain 1 to 65,535 samples.
    pub async fn write<W: Word>(&mut self, data: &[W]) {
        self.assert_dma_transfer(data.len());
        self.start_dma(false, data).await;
    }

    /// Continuously repeat a sample buffer using circular DMA.
    ///
    /// This future intentionally remains pending until it is cancelled. The channel must use an
    /// external trigger. Buffers must contain 1 to 65,535 samples.
    pub async fn write_circular<W: Word>(&mut self, data: &[W]) {
        self.assert_dma_transfer(data.len());
        self.start_dma(true, data).await;
    }

    fn assert_dma_transfer(&self, len: usize) {
        assert!(
            matches!(self.trigger, Some(trigger) if trigger.drives_dma()),
            "DAC DMA requires an external trigger"
        );
        assert!(
            (1..=u16::MAX as usize).contains(&len),
            "invalid DAC DMA buffer length"
        );
    }

    async fn start_dma<W: Word>(&mut self, circular: bool, data: &[W]) {
        let regs = self.info.regs;
        let index = self.index;

        // DMA underrun flags are cleared by writing one.
        regs.sr().write(|w| w.set_dmaudr(index, true));
        let options = TransferOptions {
            circular,
            peripheral_word_size: Some(WordSize::FourBytes),
            half_transfer_ir: false,
            complete_transfer_ir: !circular,
            ..Default::default()
        };

        let dma = self.dma.as_mut().expect("DAC channel has no DMA channel");
        let transfer = unsafe { dma.write(W::dma_buf(data), W::dma_ptr(regs, index), options) };

        // Arm DMA before allowing the DAC to emit requests. The guard disables further DAC
        // requests when the operation completes or its future is cancelled.
        let _guard = OnDrop::new(move || {
            regs.cr().modify(|w| w.set_dmaen(index, false));
        });
        regs.cr().modify(|w| {
            w.set_en(index, true);
            w.set_dmaen(index, true);
        });

        transfer.await;
    }
}

impl<'d, M: PeriMode> DacChannel<'d, M> {
    #[cfg(dma)]
    fn new_inner<T: Instance, C: Channel>(dma: Option<ChannelAndRequest<'d>>) -> Self {
        Self::new_inner_common::<T, C>(dma)
    }

    #[cfg(not(dma))]
    fn new_inner<T: Instance, C: Channel>(_dma: Option<()>) -> Self {
        Self::new_inner_common::<T, C>()
    }

    #[cfg(dma)]
    fn new_inner_common<T: Instance, C: Channel>(dma: Option<ChannelAndRequest<'d>>) -> Self {
        let info = T::info();
        let index = C::INDEX;
        info.regs.cr().modify(|w| {
            w.set_en(index, false);
            w.set_boff(index, false);
            w.set_ten(index, false);
            w.set_wave(index, vals::Wave::DISABLED);
            w.set_dmaen(index, false);
            w.set_dmaudrie(index, false);
        });

        let mut channel = Self {
            info,
            state: T::state(),
            index,
            trigger: None,
            dma,
            _mode: PhantomData,
        };
        channel.enable();
        channel
    }

    #[cfg(not(dma))]
    fn new_inner_common<T: Instance, C: Channel>() -> Self {
        let info = T::info();
        let index = C::INDEX;
        info.regs.cr().modify(|w| {
            w.set_en(index, false);
            w.set_boff(index, false);
            w.set_ten(index, false);
            w.set_wave(index, vals::Wave::DISABLED);
            w.set_dmaen(index, false);
            w.set_dmaudrie(index, false);
        });

        let mut channel = Self {
            info,
            state: T::state(),
            index,
            trigger: None,
            _mode: PhantomData,
        };
        channel.enable();
        channel
    }

    /// Enable or disable this channel.
    pub fn set_enable(&mut self, enabled: bool) {
        critical_section::with(|_| {
            self.info
                .regs
                .cr()
                .modify(|w| w.set_en(self.index, enabled));
        });
    }

    /// Enable this channel.
    pub fn enable(&mut self) {
        self.set_enable(true);
    }

    /// Disable this channel.
    pub fn disable(&mut self) {
        self.set_enable(false);
    }

    /// Configure or disable channel triggering.
    ///
    /// The channel is temporarily disabled because PY32 does not permit changing `TSEL` while
    /// the channel is enabled. Its previous enable state is restored before this method returns.
    pub fn set_trigger(&mut self, trigger: Option<Trigger>) {
        critical_section::with(|_| {
            let cr = self.info.regs.cr();
            let was_enabled = cr.read().en(self.index);
            cr.modify(|w| w.set_en(self.index, false));
            cr.modify(|w| {
                if let Some(trigger) = trigger {
                    w.set_tsel(self.index, trigger.tsel());
                    w.set_ten(self.index, true);
                } else {
                    w.set_ten(self.index, false);
                }
            });
            cr.modify(|w| w.set_en(self.index, was_enabled));
        });
        self.trigger = trigger;
    }

    /// Issue a software trigger.
    ///
    /// The channel must first be configured with `Some(Trigger::Software)`.
    pub fn trigger(&mut self) {
        assert_eq!(self.trigger, Some(Trigger::Software));
        self.info
            .regs
            .swtrigr()
            .write(|w| w.set_swtrig(self.index, true));
    }

    /// Write a new sample into this channel's holding register.
    ///
    /// With triggering disabled the output updates automatically. With triggering enabled, the
    /// new value is transferred to the output register by the next selected trigger.
    pub fn set<W: Word>(&mut self, value: W) {
        W::set_value(self.info.regs, self.index, value);
    }

    /// Read the current 12-bit output register value.
    pub fn read(&self) -> u16 {
        self.info.regs.dor(self.index).read().dor()
    }
}

impl<'d, M: PeriMode> Drop for DacChannel<'d, M> {
    fn drop(&mut self) {
        self.info.regs.cr().modify(|w| {
            w.set_dmaen(self.index, false);
            w.set_en(self.index, false);
        });
        if self.state.release() {
            self.info.rcc.disable();
        }
    }
}

/// Driver for both channels of a dual-channel DAC.
pub struct Dac<'d, M: PeriMode> {
    info: &'static Info,
    ch1: DacChannel<'d, M>,
    ch2: DacChannel<'d, M>,
}

impl<'d> Dac<'d, Blocking> {
    /// Create a blocking dual-channel DAC on its two output pins.
    pub fn new_blocking<T: Instance>(
        _peri: Peri<'d, T>,
        pin_ch1: Peri<'d, impl DacPin<T, Ch1>>,
        pin_ch2: Peri<'d, impl DacPin<T, Ch2>>,
    ) -> Self {
        pin_ch1.set_as_analog();
        pin_ch2.set_as_analog();
        rcc::enable_and_reset::<T>();
        T::state().acquire(2);

        Self {
            info: T::info(),
            ch1: DacChannel::new_inner::<T, Ch1>(None),
            ch2: DacChannel::new_inner::<T, Ch2>(None),
        }
    }
}

#[cfg(dma)]
impl<'d> Dac<'d, Async> {
    /// Create an async-capable dual-channel DAC with triggering disabled.
    pub fn new<T: Instance, D1: Dma<T, Ch1>, D2: Dma<T, Ch2>>(
        _peri: Peri<'d, T>,
        dma_ch1: Peri<'d, D1>,
        dma_ch2: Peri<'d, D2>,
        pin_ch1: Peri<'d, impl DacPin<T, Ch1>>,
        pin_ch2: Peri<'d, impl DacPin<T, Ch2>>,
    ) -> Self {
        pin_ch1.set_as_analog();
        pin_ch2.set_as_analog();
        rcc::enable_and_reset::<T>();
        T::state().acquire(2);

        Self {
            info: T::info(),
            ch1: DacChannel::new_inner::<T, Ch1>(new_dma!(dma_ch1)),
            ch2: DacChannel::new_inner::<T, Ch2>(new_dma!(dma_ch2)),
        }
    }

    /// Create an async dual-channel DAC with independent external triggers.
    pub fn new_triggered<T: Instance, D1: Dma<T, Ch1>, D2: Dma<T, Ch2>>(
        peri: Peri<'d, T>,
        dma_ch1: Peri<'d, D1>,
        dma_ch2: Peri<'d, D2>,
        trigger_ch1: Trigger,
        trigger_ch2: Trigger,
        pin_ch1: Peri<'d, impl DacPin<T, Ch1>>,
        pin_ch2: Peri<'d, impl DacPin<T, Ch2>>,
    ) -> Self {
        assert!(
            trigger_ch1.drives_dma() && trigger_ch2.drives_dma(),
            "software triggers cannot drive DAC DMA"
        );
        let mut dac = Self::new(peri, dma_ch1, dma_ch2, pin_ch1, pin_ch2);
        dac.ch1.set_trigger(Some(trigger_ch1));
        dac.ch2.set_trigger(Some(trigger_ch2));
        dac
    }
}

impl<'d, M: PeriMode> Dac<'d, M> {
    /// Split this DAC into independently movable channel handles.
    pub fn split(self) -> (DacChannel<'d, M>, DacChannel<'d, M>) {
        (self.ch1, self.ch2)
    }

    /// Borrow channel 1.
    pub fn ch1(&mut self) -> &mut DacChannel<'d, M> {
        &mut self.ch1
    }

    /// Borrow channel 2.
    pub fn ch2(&mut self) -> &mut DacChannel<'d, M> {
        &mut self.ch2
    }

    /// Write both holding registers simultaneously.
    pub fn set<W: Word>(&mut self, values: (W, W)) {
        W::set_values(self.info.regs, values);
    }
}
