#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

mod flash;

use cortex_m::delay::Delay;
use embedded_graphics::Drawable;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::DrawTarget;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::Text;
use embedded_hal::digital::v2::OutputPin;
use fugit::HertzU32;
use numtoa::NumToA;
use rp2040_panic_usb_boot as _;
use flash::{Flash, FLASH_ORIGIN};
use uc8151::Uc8151;
use core::mem::size_of;
use pimoroni_badger2040::hal as hal;
use pimoroni_badger2040::pac as pac;
use pimoroni_badger2040 as bsp;
use hal::Clock;
use hal::Sio;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;

#[allow(non_upper_case_globals)]
const MiB: usize = 1024 * 1024;
const FLASH_END: usize = FLASH_ORIGIN + 2 * MiB;
const FLASH_CONF_ADDR: usize = FLASH_END - size_of::<Flash<Conf>>();

#[repr(C)]
#[derive(Copy, Clone)]
struct Conf {
    number: u16,
}

impl Default for Conf {
    fn default() -> Self {
        Self {
            number: 1,
        }
    }
}

impl Conf {
    fn is_valid(&self) -> bool {
        self.number != 0 && !self.number != 0
    }
}

#[hal::entry]
fn _main() -> ! {

    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let core = pac::CorePeripherals::take().unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay: Delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut conf = {
        let conf: Conf = unsafe { Flash::read(FLASH_CONF_ADDR).value().assume_init() };

        if conf.is_valid() {
            conf
        } else {
            Default::default()
        }
    };

    led_pin.set_high().unwrap() ;
    // // Set up the pins for the e-ink display
    let _ = pins.sclk.into_mode::<hal::gpio::FunctionSpi>();
    let _ = pins.mosi.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);
    let dc = pins.inky_dc.into_push_pull_output();
    let cs = pins.inky_cs_gpio.into_push_pull_output();
    let busy = pins.inky_busy.into_pull_up_input();
    let reset = pins.inky_res.into_push_pull_output();

    let spi = spi.init(
        &mut pac.RESETS,
        &clocks.peripheral_clock,
        HertzU32::Hz(1_000_000),
        &embedded_hal::spi::MODE_0,
    );

    let mut display = Uc8151::new(spi, cs, dc, busy, reset);

    // Initialise display. Using the default LUT speed setting
    display.setup(&mut delay, uc8151::LUT::Ultrafast).unwrap();


    let mut buf = [0u8; 20];

    loop {
        delay.delay_ms(500);
        display.clear(BinaryColor::On).unwrap();
        Text::new(
            conf.number.numtoa_str(10, &mut buf),
            Point::new(50, 50),
            MonoTextStyle::new(&FONT_10X20, BinaryColor::Off),
        )
    .draw(&mut display)
    .unwrap();

    display.update().unwrap();

    conf.number = conf.number + 1;

    cortex_m::interrupt::free(|_cs| unsafe {
        Flash::new(conf).write(FLASH_CONF_ADDR)
    });


    }
}