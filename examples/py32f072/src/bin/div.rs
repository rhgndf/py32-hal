#![no_std]
#![no_main]

use core::hint::black_box;

use cortex_m::peripheral::{SYST, syst::SystClkSource};
use defmt::*;
use embassy_executor::Spawner;
use py32_hal::div::Div;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    const ITERATIONS: u32 = 8192;
    const SYSTICK_MASK: u32 = 0x00ff_ffff;

    let mut systick = cortex_m::Peripherals::take().unwrap().SYST;
    let p = py32_hal::init(Default::default());
    let mut div = Div::new(p.DIV);

    systick.set_clock_source(SystClkSource::Core);
    systick.set_reload(SYSTICK_MASK);
    systick.clear_current();
    systick.disable_interrupt();
    systick.enable_counter();
    while SYST::get_current() == 0 {}

    let (software_cycles, software_checksum, hardware_cycles, hardware_checksum) =
        cortex_m::interrupt::free(|_| {
            let start = SYST::get_current();
            let software_checksum = benchmark_software_division(ITERATIONS);
            let software_cycles = elapsed_cycles(start);

            let start = SYST::get_current();
            let hardware_checksum = benchmark_hardware_division(&mut div, ITERATIONS);
            let hardware_cycles = elapsed_cycles(start);

            (
                software_cycles,
                software_checksum,
                hardware_cycles,
                hardware_checksum,
            )
        });

    if software_checksum != hardware_checksum {
        defmt::panic!(
            "checksum mismatch: software={:#x}, hardware={:#x}",
            software_checksum,
            hardware_checksum
        );
    }

    info!("benchmarked {} unsigned divisions", ITERATIONS);
    info!(
        "built-in /: {} cycles, {} cycles/div",
        software_cycles,
        software_cycles / ITERATIONS
    );
    info!(
        "hardware DIV: {} cycles, {} cycles/div",
        hardware_cycles,
        hardware_cycles / ITERATIONS
    );

    let speedup_tenths = software_cycles * 10 / hardware_cycles;
    info!(
        "hardware speedup: {}.{}x (checksum={:#x})",
        speedup_tenths / 10,
        speedup_tenths % 10,
        hardware_checksum
    );
}

fn benchmark_software_division(iterations: u32) -> u32 {
    let mut checksum = 0u32;

    for iteration in 0..iterations {
        let (dividend, divisor) = operands(iteration);
        let quotient = black_box(dividend) / black_box(divisor);
        checksum = black_box(checksum.rotate_left(5) ^ quotient);
    }

    checksum
}

fn benchmark_hardware_division(div: &mut Div<'_>, iterations: u32) -> u32 {
    let mut checksum = 0u32;

    for iteration in 0..iterations {
        let (dividend, divisor) = operands(iteration);
        let (quotient, _) = div.divide_unsigned(black_box(dividend), black_box(divisor));
        checksum = black_box(checksum.rotate_left(5) ^ quotient);
    }

    checksum
}

fn operands(iteration: u32) -> (u32, u32) {
    const OPERANDS: [(u32, u32); 8] = [
        (0x7250_a3fb, 0x257d),
        (0xffff_ffff, 3),
        (0x8000_0000, 0x101),
        (0xdead_beef, 0xbeef),
        (0x1234_5678, 0x1235),
        (0xc001_cafe, 0x1fff),
        (0x7fff_ffff, 17),
        (0x0102_0304, 0x0103),
    ];

    OPERANDS[(iteration as usize) & (OPERANDS.len() - 1)]
}

fn elapsed_cycles(start: u32) -> u32 {
    start.wrapping_sub(SYST::get_current()) & 0x00ff_ffff
}
