//! # Badger2040 Blinky Example
//!
//! Blinks the activity LED on a badger2040 board, using an RP2040 Timer in Count-down mode.
//!
//! See the `Cargo.toml` file for Copyright and licence details.
#![allow(dead_code)]
#![no_std]
#![no_main]
use embedded_graphics::mono_font::ascii::FONT_4X6;
use fugit::HertzU32;
use fugit::MicrosDurationU32;
use fugit::RateExtU32;
use hal::Clock;
use pimoroni_badger2040 as bsp;

use hal::Spi;
// The macro for our start-up function
use bsp::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use bsp::hal::pac;
use bsp::hal::Timer;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use bsp::hal;

// A few traits required for using the CountDown timer
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle,
    },
    text::{Alignment, Text},
};
use uc8151::HEIGHT;
use uc8151::Uc8151;
use uc8151::WIDTH;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let _clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the timer peripheral to be a CountDown timer for our blinky delay
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();


    // Set up the pins for the e-ink display
    let _ = pins.sclk.into_mode::<hal::gpio::FunctionSpi>();
    let _ = pins.mosi.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);
    let dc = pins.inky_dc.into_push_pull_output();
    let cs = pins.inky_cs_gpio.into_push_pull_output();
    let busy = pins.inky_busy.into_pull_up_input();
    let reset = pins.inky_res.into_push_pull_output();

    let spi = spi.init(
        &mut pac.RESETS,
        &_clocks.peripheral_clock,
        HertzU32::Hz(1_000_000),
        &embedded_hal::spi::MODE_0,
    );
    let mut count_down = timer.count_down();

    let mut display = Uc8151::new(spi, cs, dc, busy, reset);
    // Reset display
    display.disable();
    count_down.start(MicrosDurationU32::micros(10));
    let _ = nb::block!(count_down.wait());
    display.enable();
    count_down.start(MicrosDurationU32::micros(10));
    let _ = nb::block!(count_down.wait());
    // Wait for the screen to finish reset
    while display.is_busy() {}

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, _clocks.system_clock.freq().to_Hz());

    // Initialise display. Using the default LUT speed setting
    display.setup(&mut delay, uc8151::LUT::Ultrafast).unwrap();

    let mut current = 0i32;
    let mut channel = 0;

    display.update().unwrap();

    loop {
        let bounds = Rectangle::new(Point::new(0, current), Size::new(WIDTH, 8));

        bounds
            .into_styled(
                PrimitiveStyleBuilder::default()
                    .stroke_color(BinaryColor::Off)
                    .fill_color(BinaryColor::On)
                    .stroke_width(1)
                    .build(),
            )
            .draw(&mut display)
            .unwrap();

        Text::new(
            value_text(channel),
            bounds.center() + Point::new(0, 2),
            MonoTextStyle::new(&FONT_4X6, BinaryColor::Off),
        )
        .draw(&mut display)
        .unwrap();

        display.partial_update(bounds.try_into().unwrap()).unwrap();

        current = (current + 8) % HEIGHT as i32;
        channel = (channel + 1) % 16;

        count_down.start(MicrosDurationU32::millis(50));
        let _ = nb::block!(count_down.wait());
    }

    // // Blink the LED at 1 Hz
    // loop {
    //     // LED on, and wait for 500ms
    //     led_pin.set_high().unwrap();
    //     delay.start(100.millis());
    //     let _ = nb::block!(delay.wait());

    //     // LED off, and wait for 500ms
    //     led_pin.set_low().unwrap();
    //     delay.start(900.millis());
    //     let _ = nb::block!(delay.wait());
    // }
}

fn value_text(value: i32) -> &'static str {
    const CHANNEL_NUM: &[&str] = &[
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15",
    ];

    #[allow(clippy::cast_sign_loss)]
    CHANNEL_NUM[(value % 16) as usize]
}