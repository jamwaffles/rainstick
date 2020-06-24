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
use embedded_hal::digital::v2::OutputPin;
use mpu9250::Mpu9250;
use panic_halt as _;
use rtic::app;
use st7789::{Orientation, ST7789};
use stm32f4xx_hal::stm32::{I2C1, SPI1, SPI2};
use stm32f4xx_hal::{
    delay::Delay,
    gpio,
    i2c::{self, I2c},
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

// SPI accel
type AccelMOSI = gpio::gpioa::PA7<gpio::Alternate<gpio::AF5>>;
type AccelMISO = gpio::gpioa::PA6<gpio::Alternate<gpio::AF5>>;
type AccelCLK = gpio::gpioa::PA5<gpio::Alternate<gpio::AF5>>;
type AccelCS = gpio::gpioa::PA4<gpio::Output<gpio::PushPull>>;

// I2C accel
type AccelSCL = gpio::gpiob::PB6<gpio::AlternateOD<gpio::AF4>>;
type AccelSDA = gpio::gpiob::PB7<gpio::AlternateOD<gpio::AF4>>;

type StatusLED = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

type Display =
    ST7789<SPIInterfaceNoCS<spi::Spi<SPI2, (DispSCK, DispMISO, DispMOSI)>, DispDC>, DispReset>;

// SPI
// type Accel = mpu9250::Mpu9250<
//     mpu9250::SpiDevice<Spi<SPI1, (AccelCLK, AccelMISO, AccelMOSI)>, AccelCS>,
//     // mpu9250::Marg,
//     mpu9250::Imu,
// >;

// I2C
type Accel =
    mpu9250::Mpu9250<mpu9250::I2cDevice<i2c::I2c<I2C1, (AccelSCL, AccelSDA)>>, mpu9250::Marg>;

const RUST: Rgb565 = Rgb565::new(0xff, 0x07, 0x00);

pub struct NoChipSelect;

impl OutputPin for NoChipSelect {
    type Error = ();

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        display: Display,
        timer: Timer<TIM2>,
        status: StatusLED,
        #[init(0)]
        count: u32,
        accel: Accel,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let dp = cx.device;
        let core = cx.core;

        let mut rcc = dp.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25u32.mhz()).sysclk(100u32.mhz()).freeze();

        let mut gpiob = dp.GPIOB.split();
        let mut gpioa = dp.GPIOA.split();
        let mut gpioc = dp.GPIOC.split();

        let mut delay = Delay::new(core.SYST, clocks);

        let mut status = gpioc.pc13.into_push_pull_output();

        let mut display = {
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

            display
        };

        // // SPI
        // let accel = {
        //     let ncs = gpioa.pa4.into_push_pull_output();
        //     let sck = gpioa.pa5.into_alternate_af5();
        //     let miso = gpioa.pa6.into_alternate_af5();
        //     let mosi = gpioa.pa7.into_alternate_af5();

        //     let spi = Spi::spi1(
        //         dp.SPI1,
        //         (sck, miso, mosi),
        //         Mode {
        //             polarity: Polarity::IdleHigh,
        //             phase: Phase::CaptureOnSecondTransition,
        //         },
        //         100.khz().into(),
        //         clocks,
        //     );

        //     let mpu =
        //         Mpu9250::imu_default(spi, ncs, &mut delay).expect("Failed to initialise MPU9250");

        //     hprintln!(
        //         "MPU9250 Initialised: {}. {}",
        //         mpu.imu_supported(),
        //         mpu.marg_supported()
        //     );

        //     mpu
        // };

        // I2C
        let accel = {
            let scl = gpiob.pb6.into_alternate_af4_open_drain();
            let sda = gpiob.pb7.into_alternate_af4_open_drain();

            let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz().into(), clocks);

            hprintln!("Initialise MPU9250...");

            let mut mpu =
                Mpu9250::marg_default(i2c, &mut delay).expect("Failed to initialise MPU9250");

            let who_am_i = mpu.who_am_i().expect("could not read WHO_AM_I");
            let mag_who_am_i = mpu
                .ak8963_who_am_i()
                .expect("could not read magnetometer's WHO_AM_I");

            mpu
        };

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
            accel,
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

        display.clear(Rgb565::BLACK).unwrap();
        Text::new(&buf, Point::zero())
            .into_styled(TextStyle::new(Font6x8, RUST))
            .draw(display)
            .unwrap();

        *count += 1;

        timer.clear_interrupt(Event::TimeOut);
    }

    extern "C" {
        fn EXTI0();
    }
};
