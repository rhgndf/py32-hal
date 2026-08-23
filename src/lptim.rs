//! Low-power timer (LPTIM)
//!
//! A 16-bit up counter clocked by an independent kernel clock (PCLK, LSI or
//! LSE, selected via `config.rcc.mux.lptim1sel`). The autoreload match flag is
//! set when the counter reaches the autoreload value; in continuous mode the
//! counter restarts, in single mode it stops.
//!
//! The PY32F072 LPTIM is a reduced STM32L0-style LPTIM: no compare register,
//! no CMPM/CMPOK flags, no external trigger inputs.

// The following code is modified from embassy-stm32
// https://github.com/embassy-rs/embassy/tree/main/embassy-stm32
// Special thanks to the Embassy Project and its contributors for their work!

use core::future::poll_fn;
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt as _;
use crate::interrupt::typelevel;
use crate::rcc;

/// LPTIM clock prescaler
pub type Prescaler = crate::pac::lptim1::vals::Presc;

/// LPTIM operating mode
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Single: the counter stops at the autoreload value.
    Single,
    /// Continuous: the counter restarts at the autoreload value.
    Continuous,
}

/// LPTIM configuration
#[derive(Clone, Copy)]
pub struct Config {
    /// Clock prescaler
    pub prescaler: Prescaler,
    /// Operating mode
    pub mode: Mode,
    /// Registers update mode: when set, CFGR/ARR updates are applied at the
    /// next counter overflow instead of immediately.
    pub preload: bool,
    /// Enable the autoreload match interrupt (needed for [`Lptim::wait_match`]).
    pub interrupt: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            prescaler: Prescaler::DIV1,
            mode: Mode::Continuous,
            preload: false,
            interrupt: true,
        }
    }
}

static WAKER: AtomicWaker = AtomicWaker::new();

/// Interrupt handler.
///
/// Wakes the tasks waiting in [`Lptim::wait_match`].
pub struct InterruptHandler;

impl typelevel::Handler<typelevel::TIM6_LPTIM1_DAC> for InterruptHandler {
    unsafe fn on_interrupt() {
        WAKER.wake();
    }
}

/// LPTIM driver
pub struct Lptim<'d> {
    _peri: Peri<'d, crate::peripherals::LPTIM1>,
    mode: Mode,
}

impl<'d> Lptim<'d> {
    /// Create a new LPTIM driver.
    ///
    /// The kernel clock must be running: enable LSI or LSE in
    /// `config.rcc.ls` and select it with `config.rcc.mux.lptim1sel`, or use
    /// PCLK (the default mux selection).
    pub fn new(
        peri: Peri<'d, crate::peripherals::LPTIM1>,
        _irq: impl typelevel::Binding<typelevel::TIM6_LPTIM1_DAC, InterruptHandler>,
        config: Config,
    ) -> Self {
        assert!(
            rcc::frequency::<crate::peripherals::LPTIM1>().0 != 0,
            "LPTIM1 kernel clock is not running: enable it in config.rcc.ls and select it in config.rcc.mux"
        );

        rcc::enable_and_reset::<crate::peripherals::LPTIM1>();

        // CFGR and IER can only be modified while the LPTIM is disabled.
        let lptim = crate::pac::LPTIM1;
        lptim.cfgr().write(|w| {
            w.set_presc(config.prescaler);
            w.set_preload(config.preload);
        });
        lptim.ier().write(|w| w.set_arrmie(config.interrupt));

        lptim.cr().modify(|w| w.set_enable(true));

        typelevel::TIM6_LPTIM1_DAC::unpend();
        unsafe { typelevel::TIM6_LPTIM1_DAC::enable() };

        Self {
            _peri: peri,
            mode: config.mode,
        }
    }

    /// Set the autoreload value.
    ///
    /// The LPTIM must be enabled (it is by default). The write completes when
    /// the ARROK flag is set, which is then cleared.
    pub fn set_autoreload(&mut self, arr: u16) {
        let lptim = crate::pac::LPTIM1;
        lptim.arr().write(|w| w.set_arr(arr));
        while !lptim.isr().read().arrok() {}
        lptim.icr().write(|w| w.set_arrokcf(true));
    }

    /// Read the autoreload value.
    pub fn autoreload(&self) -> u16 {
        crate::pac::LPTIM1.arr().read().arr()
    }

    /// Read the counter value.
    ///
    /// Two consecutive reads must return the same value for the result to be
    /// valid (RM 26.4.3), so this loops until that holds.
    pub fn counter(&self) -> u16 {
        let lptim = crate::pac::LPTIM1;
        loop {
            let a = lptim.cnt().read().cnt();
            let b = lptim.cnt().read().cnt();
            if a == b {
                return a;
            }
        }
    }

    /// Start the LPTIM.
    ///
    /// In continuous mode the counter counts up to the autoreload value and
    /// restarts; in single mode it stops at the autoreload value.
    pub fn start(&mut self) {
        let lptim = crate::pac::LPTIM1;
        lptim.cr().modify(|w| {
            w.set_enable(true);
            match self.mode {
                Mode::Single => w.set_sngstrt(true),
                Mode::Continuous => w.set_cntstrt(true),
            }
        });
    }

    /// Stop the LPTIM.
    pub fn stop(&mut self) {
        crate::pac::LPTIM1.cr().modify(|w| w.set_enable(false));
    }

    /// Reset the counter. The LPTIM must be enabled.
    pub fn reset_counter(&mut self) {
        let lptim = crate::pac::LPTIM1;
        lptim.cr().modify(|w| w.set_countrst(true));
        while lptim.cr().read().countrst() {}
    }

    /// Check whether the autoreload match flag is set.
    pub fn match_flag(&self) -> bool {
        crate::pac::LPTIM1.isr().read().arrm()
    }

    /// Clear the autoreload match flag.
    pub fn clear_match_flag(&mut self) {
        crate::pac::LPTIM1.icr().write(|w| w.set_arrmcf(true));
    }

    /// Wait for the next autoreload match.
    ///
    /// The autoreload match interrupt must be enabled (see
    /// [`Config::interrupt`]).
    pub async fn wait_match(&mut self) {
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            let lptim = crate::pac::LPTIM1;
            if lptim.isr().read().arrm() {
                lptim.icr().write(|w| w.set_arrmcf(true));
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}
