//! Bounce a DVD player logo around the screen
//!
//! Like this, but with no color changing: https://bouncingdvdlogo.com/
//!
//! For best results, run with the `--release` flag.

#![no_std]
#![no_main]

use core::convert::TryFrom;
use cortex_m_semihosting::hprintln;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::{
    fonts::Text, geometry::Point, image::Image, pixelcolor::BinaryColor, prelude::*, primitives::*,
    style::*,
};
use panic_halt as _;
use rtic::app;
use st7789::{Orientation, ST7789};
use stm32f4xx_hal::stm32::{SPI1, SPI2};
use stm32f4xx_hal::{
    delay::Delay,
    gpio,
    prelude::*,
    spi::{self, Mode, Phase, Polarity, Spi},
    stm32::{self, TIM2},
    time::Hertz,
    timer::{Event, Timer},
};

type DispSCK = gpio::gpiob::PB13<gpio::Alternate<gpio::AF5>>;
type DispMISO = spi::NoMiso;
type DispMOSI = gpio::gpiob::PB15<gpio::Alternate<gpio::AF5>>;
type DispReset = gpio::gpioa::PA8<gpio::Output<gpio::PushPull>>;
type DispDC = gpio::gpioa::PA9<gpio::Output<gpio::PushPull>>;

type StatusLED = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

type Display =
    ST7789<SPIInterfaceNoCS<spi::Spi<SPI2, (DispSCK, DispMISO, DispMOSI)>, DispDC>, DispReset>;

const RUST: Rgb565 = Rgb565::new(0xff, 0x07, 0x00);

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        display: Display,
        timer: Timer<TIM2>,
        status: StatusLED,
        #[init(0)]
        count: u32,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let dp = cx.device;
        let core = cx.core;

        let mut rcc = dp.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(25u32.mhz())
            .hclk(100u32.mhz())
            .sysclk(100u32.mhz())
            .pclk1(72u32.mhz())
            .pclk2(72u32.mhz())
            .freeze();

        let mut gpiob = dp.GPIOB.split();
        let mut gpioa = dp.GPIOA.split();
        let mut gpioc = dp.GPIOC.split();

        let mut delay = Delay::new(core.SYST, clocks);

        let mut status = gpioc.pc13.into_push_pull_output();

        let sck = gpiob.pb13.into_alternate_af5();
        let mosi = gpiob.pb15.into_alternate_af5();

        let mut rst = gpioa.pa8.into_push_pull_output();
        let dc = gpioa.pa9.into_push_pull_output();

        let spi = Spi::spi2(
            dp.SPI2,
            (sck, spi::NoMiso, mosi),
            Mode {
                polarity: Polarity::IdleHigh,
                phase: Phase::CaptureOnFirstTransition,
            },
            48.mhz().into(),
            clocks,
        );

        hprintln!("Spi setup");
        status.toggle();

        let interface = display_interface_spi::SPIInterfaceNoCS::new(spi, dc);

        hprintln!("Interface initialised");
        status.toggle();

        let mut display = ST7789::new(interface, rst, 240, 240);

        display.init(&mut delay).unwrap();

        hprintln!("Display initialised");
        status.toggle();

        // display.set_orientation(Orientation::Landscape).unwrap();

        // Update framerate
        let fps: u32 = 1;

        let mut timer = Timer::tim2(dp.TIM2, fps.hz(), clocks);

        timer.listen(Event::TimeOut);

        let circle1 = Circle::new(Point::new(128, 64), 64)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
        let circle2 = Circle::new(Point::new(64, 64), 64)
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1));

        let blue_with_red_outline = PrimitiveStyleBuilder::new()
            .fill_color(Rgb565::BLUE)
            .stroke_color(Rgb565::RED)
            .stroke_width(1) // > 1 is not currently supported in embedded-graphics on triangles
            .build();
        let triangle = Triangle::new(
            Point::new(40, 120),
            Point::new(40, 220),
            Point::new(140, 120),
        )
        .into_styled(blue_with_red_outline);

        // draw two circles on black background
        display.clear(Rgb565::BLACK).unwrap();
        circle1.draw(&mut display).unwrap();
        circle2.draw(&mut display).unwrap();
        triangle.draw(&mut display).unwrap();

        hprintln!("Init complete");
        status.toggle();

        // Init the static resources to use them later through RTIC
        init::LateResources {
            timer,
            display,
            status,
        }
    }

    #[task(binds = TIM2, resources = [timer, display, count, status])]
    fn update(cx: update::Context) {
        use core::fmt::Write;
        use heapless::consts::U16;

        let update::Resources {
            timer,
            display,
            count,
            status,
            ..
        } = cx.resources;

        let mut buf = heapless::String::<U16>::new();

        status.toggle();

        write!(&mut buf, "Count: {}", count);

        Text::new(&buf, Point::zero())
            .into_styled(TextStyle::new(Font6x8, RUST))
            .draw(display)
            .unwrap();

        *count += 1;
    }

    extern "C" {
        fn EXTI0();
    }
};
