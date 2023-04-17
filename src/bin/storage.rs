//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use rp2040_panic_usb_boot as _;
use tickv::FlashController;

use core::hash::Hash;
use core::hash::Hasher;
use hal::spi::Enabled;
use siphasher::sip::SipHasher;

use badger2040_rust::{RP2040FlashCtrl, SECTOR_SIZE};
use cortex_m::prelude::_embedded_hal_timer_CountDown;
use embedded_graphics::{text::Text, prelude::Point, mono_font::{MonoTextStyle, ascii::FONT_4X6, iso_8859_16::FONT_10X20}, pixelcolor::BinaryColor, Drawable};
use fugit::{HertzU32, MicrosDurationU32};
use hal::Timer;
use numtoa::NumToA;
use pimoroni_badger2040::hal as hal;
use embedded_hal::digital::v2::OutputPin;


// How big is your flash? Default for this example is 2MiB
pub const FLASH_SIZE_MBYTES: u32 = 2;
// How much space to use for this test?
// SECTOR_SIZE is 4096 so 16*4096 is 64KiB:
pub const STORAGE_SIZE: u32 = 16 * SECTOR_SIZE as u32;

pub const FLASH_END_ADDR: u32 = FLASH_SIZE_MBYTES * 1024 * 1024;
pub const STORAGE_ADDR: u32 = FLASH_END_ADDR - STORAGE_SIZE;

// These are here mostly for reference; they're 'block_cmd' you can pass to
// flash_range_erase() (as the 4th arg) in order to speed up erasure operations.
// They're specific to the brand/type of flash chip used so we don't use them
// by default...
pub const PAGE_ERASE: u8 = 0x02;
pub const SECTOR_ERASE: u8 = 0x20;
pub const BLOCK32_ERASE: u8 = 0x52;
pub const BLOCK64_ERASE: u8 = 0xD8;


// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use pimoroni_badger2040 as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
}, entry};
use tickv::ErrorCode;
use tickv::success_codes::SuccessCode;
use tickv::{TicKV, MAIN_KEY};
use uc8151::Uc8151;
use rp2040_hal::Spi;
type DisplayType = Uc8151<Spi<Enabled, pac::SPI0, 8>, hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::Pin<hal::gpio::bank0::Gpio20, hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::Pin<hal::gpio::bank0::Gpio26, hal::gpio::Input<hal::gpio::PullUp>>,
    hal::gpio::Pin<hal::gpio::bank0::Gpio21, hal::gpio::Output<hal::gpio::PushPull>>>;
#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();


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

    // Initialise display. Using the default LUT speed setting
    display.setup(&mut delay, uc8151::LUT::Ultrafast).unwrap();



    // Setup our TicKV stuff
    let controller = RP2040FlashCtrl::new(FLASH_END_ADDR, STORAGE_SIZE).unwrap();
    let mut storage_buffer = &mut [0; SECTOR_SIZE];
    for (i, &byte) in b"Test".iter().enumerate() {
        storage_buffer[i] = byte;
    }
    controller.write(0, storage_buffer).unwrap();
    let tickv = TicKV::<RP2040FlashCtrl, { SECTOR_SIZE }>::new(
        controller,
        &mut storage_buffer,
        STORAGE_SIZE as usize,
    );

    let mut hasher = SipHasher::new();
    MAIN_KEY.hash(&mut hasher);
    let res = hasher.finish();
    led_pin.set_high().unwrap();
    tickv.initialise(res).handle_error(&mut display).unwrap();
    led_pin.set_low().unwrap();

    // Collect the garbage in case of subsequent calls to ensure there's always
    // room to store our four test keys/values:
    tickv.garbage_collect().unwrap();
    let mut buf: [u8; 8] = [0; 8];
    let mut number = 10;

    match tickv.get_key(get_hashed_key(b"test"), &mut buf) {
    Ok(_) => number = buf[0],
    Err(_) => number = 5,
    }
    number = number + 1;
    buf[0] = number;
    tickv.append_key(get_hashed_key(b"test"), &mut buf).unwrap();

    let mut buf = [0u8; 20];

    Text::new(
        number.numtoa_str(10, &mut buf),
        Point::new(50, 50),
        MonoTextStyle::new(&FONT_10X20, BinaryColor::Off),
    )
    .draw(&mut display)
    .unwrap();

    display.update().unwrap();


    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

fn get_hashed_key(unhashed_key: &[u8]) -> u64 {
    let mut hash_function = SipHasher::new();
    unhashed_key.hash(&mut hash_function);
    hash_function.finish()
}

trait HandleErrorExt {
    fn handle_error(&self, display: &mut DisplayType) -> Self;
}
impl HandleErrorExt for Result<SuccessCode, ErrorCode> {

fn handle_error(&self, display: &mut DisplayType) -> Self {
    match self {
        Ok(_) => *self,
        Err(err) =>
        {
            let text = match err {
                ErrorCode::UnsupportedVersion => "UnsupportedVersion",
                ErrorCode::CorruptData => "CorruptData",
                ErrorCode::InvalidCheckSum => "InvalidCheckSum",
                ErrorCode::KeyNotFound => "KeyNotFound",
                ErrorCode::KeyAlreadyExists => "KeyAlreadyExists",
                ErrorCode::RegionFull => "RegionFull",
                ErrorCode::FlashFull => "FlashFull",
                ErrorCode::ReadFail => "ReadFail",
                ErrorCode::WriteFail => "WriteFail",
                ErrorCode::EraseFail => "EraseFail",
                ErrorCode::ObjectTooLarge => "ObjectTooLarge",
                ErrorCode::BufferTooSmall(_) => "BufferTooSmall",
                ErrorCode::ReadNotReady(_) => "ReadNotReady",
                ErrorCode::WriteNotReady(_) => "WriteNotReady",
                ErrorCode::EraseNotReady(_) => "EraseNotReady",
            };
            Text::new("test",
        Point::new(50, 50),
        MonoTextStyle::new(&FONT_10X20, BinaryColor::Off),
    )
    .draw(display)
    .unwrap();
display.update().unwrap();
return *self;
        },
    }
}
}

// End of file