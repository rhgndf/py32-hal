//! Real-time clock (RTC)
//!
//! The PY32F072 RTC is a classic F1-style peripheral: a 20-bit programmable
//! prescaler (PRL), a 32-bit up counter (CNT) and a 32-bit alarm register
//! (ALR). The alarm fires when the counter reaches the alarm value. A second
//! flag is set once per prescaler cycle and an overflow flag when the counter
//! wraps.
//!
//! The RTC core is powered from the backup domain: writing RTC registers
//! requires disabling the backup domain write protection (PWR_CR1.DBP) first,
//! which this driver does automatically. Writes to PRL, CNT, ALR and BKP_RTCCR
//! must be done in configuration mode (CNF sequence); every RTC register write
//! must wait until the previous one completed (RTOFF).

// The following code is modified from embassy-stm32
// https://github.com/embassy-rs/embassy/tree/main/embassy-stm32
// Special thanks to the Embassy Project and its contributors for their work!

use core::future::poll_fn;
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt as _;
use crate::interrupt::typelevel;
use crate::pac::rcc::vals::Rtcsel;
use crate::pac::rtc::Rtc as Regs;
use crate::rcc;

/// RTC clock source
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtcClockSource {
    /// Low-speed external oscillator (LSE)
    Lse,
    /// Low-speed internal oscillator (LSI), 32.768 kHz
    Lsi,
    /// High-speed external oscillator divided by 128 (HSE/128)
    HseDiv128,
}

impl RtcClockSource {
    fn to_vals(self) -> Rtcsel {
        match self {
            RtcClockSource::Lse => Rtcsel::LSE,
            RtcClockSource::Lsi => Rtcsel::LSI,
            RtcClockSource::HseDiv128 => Rtcsel::HSE_DIV128,
        }
    }
}

/// RTC interrupt source
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtcInterrupt {
    /// Overflow: the counter wrapped past 0xFFFF_FFFF
    Overflow,
    /// Alarm: the counter reached the alarm value
    Alarm,
    /// Second: the prescaler completed one cycle
    Second,
}

/// RTC configuration
#[derive(Clone, Copy)]
pub struct RtcConfig {
    /// RTC clock source
    pub clock_source: RtcClockSource,
    /// Prescaler value (PRL). The RTC clock is divided by `prescaler + 1`.
    /// Must be in `1..=0xF_FFFF`. 32767 yields a 1 Hz counter tick from a
    /// 32.768 kHz clock.
    pub prescaler: u32,
}

impl Default for RtcConfig {
    fn default() -> Self {
        Self {
            clock_source: RtcClockSource::Lsi,
            prescaler: 32767,
        }
    }
}

/// Split a 20-bit prescaler into the PRLH (bits 19:16) and PRLL (bits 15:0) halves.
fn split_prescaler(prl: u32) -> (u8, u16) {
    ((prl >> 16) as u8, prl as u16)
}

static WAKER: AtomicWaker = AtomicWaker::new();

/// Interrupt handler.
///
/// Wakes the tasks waiting in [`Rtc::wait_second`] and [`Rtc::wait_alarm`].
pub struct InterruptHandler;

impl typelevel::Handler<typelevel::RTC> for InterruptHandler {
    unsafe fn on_interrupt() {
        WAKER.wake();
    }
}

/// RTC driver
pub struct Rtc<'d> {
    _peri: Peri<'d, crate::peripherals::RTC>,
}

impl<'d> Rtc<'d> {
    /// Create a new RTC driver.
    ///
    /// Enables the PWR clock and disables the backup domain write protection
    /// (PWR_CR1.DBP), resets the backup domain, enables the selected clock
    /// source, enables the RTC and programs the prescaler.
    pub fn new(
        peri: Peri<'d, crate::peripherals::RTC>,
        _irq: impl typelevel::Binding<typelevel::RTC, InterruptHandler>,
        config: RtcConfig,
    ) -> Self {
        assert!(
            config.prescaler > 0 && config.prescaler <= 0xF_FFFF,
            "invalid RTC prescaler: must be in 1..=0xFFFFF"
        );

        // Enable PWR and unprotect the backup domain so RTC registers can be written.
        rcc::enable_and_reset::<crate::peripherals::PWR>();
        crate::pac::PWR.cr1().modify(|w| w.set_dbp(true));

        // Reset the backup domain (RTC counter, RTCSEL, LSE...).
        let rcc = &crate::pac::RCC;
        rcc.bdcr().modify(|w| w.set_bdrst(true));
        rcc.bdcr().modify(|w| w.set_bdrst(false));

        // Enable the RTC clock source.
        match config.clock_source {
            RtcClockSource::Lsi => {
                rcc.csr().modify(|w| w.set_lsion(true));
                while !rcc.csr().read().lsirdy() {}
            }
            RtcClockSource::Lse => {
                rcc.bdcr().modify(|w| w.set_lseon(true));
                while !rcc.bdcr().read().lserdy() {}
            }
            // The HSE clock must be enabled in `config.rcc` by the caller.
            RtcClockSource::HseDiv128 => {}
        }

        // Select the RTC clock and enable the RTC.
        rcc.bdcr().modify(|w| {
            w.set_rtcsel(config.clock_source.to_vals());
            w.set_rtcen(true);
        });

        let rtc = crate::pac::RTC;

        // Wait for the shadow registers to be synchronized after enabling.
        while !rtc.crl().read().rsf() {}

        // Program the prescaler in configuration mode.
        let (prlh, prll) = split_prescaler(config.prescaler);
        Self::write_cnf(&rtc, |r| {
            r.prlh().write(|w| w.set_prl(prlh));
            r.prll().write(|w| w.set_prl(prll));
        });

        typelevel::RTC::unpend();
        unsafe { typelevel::RTC::enable() };

        Self { _peri: peri }
    }

    /// Read the current counter value.
    pub fn now(&self) -> u32 {
        let rtc = crate::pac::RTC;
        // Reading CNTL latches CNTH, so read the low register first.
        let lo = rtc.cntl().read().rtc_cnt() as u32;
        let hi = rtc.cnth().read().rtc_cnt() as u32;
        (hi << 16) | lo
    }

    /// Set the counter value.
    pub fn set_counter(&mut self, counter: u32) {
        let rtc = crate::pac::RTC;
        Self::write_cnf(&rtc, |r| {
            r.cnth().write(|w| w.set_rtc_cnt((counter >> 16) as u16));
            r.cntl().write(|w| w.set_rtc_cnt(counter as u16));
        });
    }

    /// Read the alarm value.
    pub fn alarm(&self) -> u32 {
        let rtc = crate::pac::RTC;
        let lo = rtc.alrl().read().rtc_alr() as u32;
        let hi = rtc.alrh().read().rtc_alr() as u32;
        (hi << 16) | lo
    }

    /// Set the alarm value. The alarm flag is set when the counter reaches it.
    pub fn set_alarm(&mut self, alarm: u32) {
        let rtc = crate::pac::RTC;
        Self::write_cnf(&rtc, |r| {
            r.alrh().write(|w| w.set_rtc_alr((alarm >> 16) as u16));
            r.alrl().write(|w| w.set_rtc_alr(alarm as u16));
        });
    }

    /// Enable an RTC interrupt.
    pub fn enable_interrupt(&mut self, i: RtcInterrupt) {
        self.set_interrupt(i, true);
    }

    /// Disable an RTC interrupt.
    pub fn disable_interrupt(&mut self, i: RtcInterrupt) {
        self.set_interrupt(i, false);
    }

    fn set_interrupt(&mut self, i: RtcInterrupt, enable: bool) {
        let rtc = crate::pac::RTC;
        // A CRH write only requires RTOFF=1.
        while !rtc.crl().read().rtoff() {}
        rtc.crh().modify(|w| match (i, enable) {
            (RtcInterrupt::Overflow, true) => w.set_owie(true),
            (RtcInterrupt::Overflow, false) => w.set_owie(false),
            (RtcInterrupt::Alarm, true) => w.set_alrie(true),
            (RtcInterrupt::Alarm, false) => w.set_alrie(false),
            (RtcInterrupt::Second, true) => w.set_secie(true),
            (RtcInterrupt::Second, false) => w.set_secie(false),
        });
    }

    /// Check whether an RTC interrupt flag is set.
    pub fn interrupt_flag(&self, i: RtcInterrupt) -> bool {
        let crl = crate::pac::RTC.crl().read();
        match i {
            RtcInterrupt::Overflow => crl.owf(),
            RtcInterrupt::Alarm => crl.alrf(),
            RtcInterrupt::Second => crl.secf(),
        }
    }

    /// Clear an RTC interrupt flag. Writing 0 to a flag clears it (RC_W0).
    pub fn clear_interrupt_flag(&mut self, i: RtcInterrupt) {
        let rtc = crate::pac::RTC;
        while !rtc.crl().read().rtoff() {}
        rtc.crl().modify(|w| match i {
            RtcInterrupt::Overflow => w.set_owf(false),
            RtcInterrupt::Alarm => w.set_alrf(false),
            RtcInterrupt::Second => w.set_secf(false),
        });
    }

    /// Wait for the next second tick.
    ///
    /// The `Second` interrupt must be enabled first with
    /// [`Rtc::enable_interrupt`].
    pub async fn wait_second(&mut self) {
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            let rtc = crate::pac::RTC;
            if rtc.crl().read().secf() {
                rtc.crl().modify(|w| w.set_secf(false));
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Wait for the alarm.
    ///
    /// The `Alarm` interrupt must be enabled first with
    /// [`Rtc::enable_interrupt`].
    pub async fn wait_alarm(&mut self) {
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            let rtc = crate::pac::RTC;
            if rtc.crl().read().alrf() {
                rtc.crl().modify(|w| w.set_alrf(false));
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Run `f` on the RTC registers in configuration mode (CNF sequence).
    ///
    /// All writes to PRL, CNT, ALR and BKP_RTCCR must be done with the CNF
    /// sequence: wait for RTOFF, set CNF, write, clear CNF, wait for RTOFF.
    fn write_cnf(rtc: &Regs, f: impl FnOnce(&Regs)) {
        let crl = &rtc.crl();
        while !crl.read().rtoff() {}
        crl.modify(|w| w.set_cnf(true));
        f(rtc);
        crl.modify(|w| w.set_cnf(false));
        while !crl.read().rtoff() {}
    }
}

#[cfg(test)]
mod tests {
    use super::split_prescaler;

    #[test]
    fn split_prescaler_bounds() {
        assert_eq!(split_prescaler(0), (0, 0));
        assert_eq!(split_prescaler(1), (0, 1));
        assert_eq!(split_prescaler(0xFFFF), (0, 0xFFFF));
        assert_eq!(split_prescaler(0x1_0000), (1, 0));
        assert_eq!(split_prescaler(0xF_FFFF), (0xF, 0xFFFF));
    }
}
