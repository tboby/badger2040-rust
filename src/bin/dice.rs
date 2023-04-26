#![no_std]
#![no_main]


use badger2040_rust::logic::dice;
use badger2040_rust::logic::dice::ElementKind;
use badger2040_rust::logic::dice::Slot;
use bsp::hal::rosc::Enabled;
use bsp::hal::rosc::RingOscillator;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use embedded_graphics::Drawable;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Dimensions;
use embedded_graphics::prelude::DrawTarget;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::Text;
use embedded_hal::digital::v2::OutputPin;
use embedded_layout::chain;
use embedded_layout::layout::linear::LinearLayout;
use embedded_layout::prelude::Align;
use embedded_layout::prelude::Chain;
use embedded_layout::prelude::Link;
use embedded_layout::prelude::horizontal;
use embedded_layout::prelude::vertical;
use embedded_layout::view_group;
use embedded_layout::view_group::Views;
use fugit::HertzU32;
use heapless::String;
use rp2040_panic_usb_boot as _;
use badger2040_rust::flash::{Flash, FLASH_ORIGIN};
use uc8151::Uc8151;
use core::array::from_fn;
use core::cell::RefCell;
use core::mem::size_of;
use pimoroni_badger2040::hal as hal;
use pimoroni_badger2040::pac as pac;
use pimoroni_badger2040 as bsp;
use hal::Clock;
use hal::Sio;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::Interrupt::EdgeLow;
use hal::pac::interrupt;
use rand::prelude::*;

#[allow(non_upper_case_globals)]
const MiB: usize = 1024 * 1024;
const FLASH_END: usize = FLASH_ORIGIN + 2 * MiB;
const FLASH_CONF_ADDR: usize = FLASH_END - size_of::<Flash<Conf>>();

#[repr(C)]
#[derive(Copy, Clone)]
struct Conf {
    elements: [Slot; 10],
    nextIndex: usize,
    checksum: u16
}
fn checksum(data: &[Slot; 10]) -> u16 {
    let mut sum: u16 = 0;

    for &item in data.iter() {
        let kind_value = match item.element_type {
            ElementKind::Blank => 0,
            ElementKind::D6 => 1,
            ElementKind::GuardianDie => 2,
            ElementKind::GrailCoin => 3,
        };

        sum += kind_value as u16;
        sum += item.value as u16;
    }

    sum & 255
}


impl Default for Conf {
    fn default() -> Self {
        Self {
            elements: [Slot{element_type: ElementKind::Blank, value: 0}; 10],
            nextIndex: 0,
            checksum: checksum(&[Slot{element_type: ElementKind::Blank, value: 0}; 10])
        }
    }
}

impl Conf {
    fn is_valid(&self) -> bool {
        checksum(&self.elements) == self.checksum
    }
}
type ButtonDown = bsp::hal::gpio::Pin<bsp::hal::gpio::bank0::Gpio11, bsp::hal::gpio::PullDownInput>;
type ButtonA = bsp::hal::gpio::Pin<bsp::hal::gpio::bank0::Gpio12, bsp::hal::gpio::PullDownInput>;
type ButtonB = bsp::hal::gpio::Pin<bsp::hal::gpio::bank0::Gpio13, bsp::hal::gpio::PullDownInput>;
type ButtonC = bsp::hal::gpio::Pin<bsp::hal::gpio::bank0::Gpio14, bsp::hal::gpio::PullDownInput>;
type ButtonUp = bsp::hal::gpio::Pin<bsp::hal::gpio::bank0::Gpio15, bsp::hal::gpio::PullDownInput>;
type Led = hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::Output<hal::gpio::PushPull>>;
type EnablePin = hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::Output<hal::gpio::PushPull>>;
type Display = Uc8151<hal::Spi<hal::spi::Enabled, pac::SPI0, 8>, hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::Output<hal::gpio::PushPull>>, hal::gpio::Pin<hal::gpio::bank0::Gpio20, hal::gpio::Output<hal::gpio::PushPull>>, hal::gpio::Pin<hal::gpio::bank0::Gpio26, hal::gpio::Input<hal::gpio::PullUp>>, hal::gpio::Pin<hal::gpio::bank0::Gpio21, hal::gpio::Output<hal::gpio::PushPull>>>;
struct BadgerGpio {
    buttonA : ButtonA,
    buttonB : ButtonB,
    buttonC : ButtonC,
    buttonUp : ButtonUp,
    buttonDown : ButtonDown,
    display : Display,
    led : Led,
    enable: EnablePin,
    rosc: RingOscillator<Enabled>
}
static GLOBAL_PINS: Mutex<RefCell<Option<BadgerGpio>>> = Mutex::new(RefCell::new(None));

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

    let mut led_pin: hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::Output<hal::gpio::PushPull>> = pins.led.into_push_pull_output();
    let mut enable: hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::Output<hal::gpio::PushPull>> = pins.p3v3_en.into_push_pull_output();
    enable.set_high().unwrap();
    led_pin.set_high().unwrap();

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

    let rosc = bsp::hal::rosc::RingOscillator::new(pac.ROSC).initialize();

    // let rng = bsp::hal::rosc::RingOscillator::gen_range(., 0..=2);

    // led_pin.set_high().unwrap() ;
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

    let mut display: Uc8151<hal::Spi<hal::spi::Enabled, pac::SPI0, 8>, hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::Output<hal::gpio::PushPull>>, hal::gpio::Pin<hal::gpio::bank0::Gpio20, hal::gpio::Output<hal::gpio::PushPull>>, hal::gpio::Pin<hal::gpio::bank0::Gpio26, hal::gpio::Input<hal::gpio::PullUp>>, hal::gpio::Pin<hal::gpio::bank0::Gpio21, hal::gpio::Output<hal::gpio::PushPull>>> = Uc8151::new(spi, cs, dc, busy, reset);

    // Initialise display. Using the default LUT speed setting
    display.setup(&mut delay, uc8151::LUT::Ultrafast).unwrap();


    let mut buf = [0u8; 20];
    let mut iters = 10;

    // conf.elements[2] = Slot{ element_type: ElementKind::GrailCoin, value: 1};
    // conf.elements[4] = Slot{ element_type: ElementKind::D6, value: 2};


    display.update().unwrap();

    pins.sw_a.set_interrupt_enabled(EdgeLow, true);
    pins.sw_b.set_interrupt_enabled(EdgeLow, true);
    pins.sw_c.set_interrupt_enabled(EdgeLow, true);


    critical_section::with(|cs| {
        GLOBAL_PINS.borrow(cs).replace(Some(
            BadgerGpio{
                buttonA: pins.sw_a.into_mode(),
        buttonB: pins.sw_b.into_mode(),
        buttonC: pins.sw_c.into_mode(),
        buttonUp: pins.sw_up.into_mode(),
        buttonDown: pins.sw_down.into_mode(),
        display: display,
        led: led_pin,
        enable: enable,
        rosc: rosc
            }));
    });
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }


    cortex_m::asm::wfi();
}

fn render(conf : &Conf, display: &mut Display){
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::Off);
    let inter =
        conf.elements
        .map(|element| element.slot_to_string());
    let items : [Text<'_, MonoTextStyle<'_, BinaryColor>>; 10] = from_fn(|idx| Text::new(inter[idx].as_str(), Point::new(0, 0), text_style));
    // let items = inter2
    //     .map(|label| Text::new(label, Point::new(0, 0), text_style));

    let chain = Chain::new(items[0])
        .append(items[1])
        .append(items[2])
        .append(items[3])
        .append(items[4])
        .append(items[5])
        .append(items[6])
        .append(items[7])
        .append(items[8])
        .append(items[9]);


        display.clear(BinaryColor::On).unwrap();
        let display_area = display.bounding_box();
            // The layout
    LinearLayout::horizontal(
        chain
            )
    .with_alignment(vertical::Center)
    .arrange()
    .align_to(&display_area, horizontal::Center, vertical::Center)
    .draw(display)
    .unwrap();
    display.update().unwrap();
}


#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndButton>`
    static mut BUTTONS: Option<BadgerGpio> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if BUTTONS.is_none() {
        critical_section::with(|cs| {
            *BUTTONS = GLOBAL_PINS.borrow(cs).take();
        });
    }

    // Need to check if our Option<LedAndButtonPins> contains our pins
    if let Some(gpios) = BUTTONS {

            let mut conf = {
        let conf: Conf = unsafe { Flash::read(FLASH_CONF_ADDR).value().assume_init() };

        if conf.is_valid() {
            conf
        } else {
            Default::default()
        }
    };

        // borrow led and button by *destructuring* the tuple
        // these will be of type `&mut LedPin` and `&mut ButtonPin`, so we don't have
        // to move them back into the static after we use them
        // Check if the interrupt source is from the pushbutton going from high-to-low.
        // Note: this will always be true in this example, as that is the only enabled GPIO interrupt source
        if gpios.buttonA.interrupt_status(EdgeLow) {
            conf.elements[conf.nextIndex] = Slot{ element_type: ElementKind::D6, value: gpios.rosc.gen_range(1..=6)};
            conf.nextIndex = (conf.nextIndex + 1) % 10;
            render(&conf, &mut gpios.display);
            gpios.buttonA.clear_interrupt(EdgeLow);
        }

        if gpios.buttonB.interrupt_status(EdgeLow) {
            conf.elements[conf.nextIndex] = Slot{ element_type: ElementKind::GuardianDie, value: gpios.rosc.gen_range(1..=6)};
            conf.nextIndex = (conf.nextIndex + 1) % 10;
            render(&conf, &mut gpios.display);
            gpios.buttonB.clear_interrupt(EdgeLow);
        }

        if gpios.buttonC.interrupt_status(EdgeLow) {
            conf.elements[conf.nextIndex] = Slot{ element_type: ElementKind::GrailCoin, value: gpios.rosc.gen_range(1..=2)};
            conf.nextIndex = (conf.nextIndex + 1) % 10;
            render(&conf, &mut gpios.display);
            gpios.buttonC.clear_interrupt(EdgeLow);
        }

        if conf.nextIndex == 0 {
            conf.elements[..5].swap_with_slice(&mut [Slot{element_type: ElementKind::Blank, value: 0}; 5]);
        }
        else if conf.nextIndex == 5 {
            conf.elements[5..].swap_with_slice(&mut [Slot{element_type: ElementKind::Blank, value: 0}; 5]);
        }

        cortex_m::interrupt::free(|_cs| unsafe {
            conf.checksum = checksum(&conf.elements);
            Flash::new(conf).write(FLASH_CONF_ADDR)
        });

        gpios.enable.set_low().unwrap();
    }
}
