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
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::{
    fonts::*, geometry::Point, image::Image, pixelcolor::BinaryColor, prelude::*, primitives::*,
    style::*,
};
use embedded_hal::digital::v2::OutputPin;
use madgwick::*;
use mpu9250::{MargMeasurements, Mpu9250};
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
type IMU =
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

const SAMPLE_RATE_HZ: u32 = 5;

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        display: Display,
        timer: Timer<TIM2>,
        status: StatusLED,
        #[init(0)]
        count: u32,
        imu: IMU,
        madgwick: Marg,
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

            display.clear(Rgb565::BLACK).unwrap();

            display
        };

        let mut madgwick = Marg::new(0.4, 1.0 / SAMPLE_RATE_HZ as f32);

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
        let mut imu = {
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

        let mut timer = Timer::tim2(dp.TIM2, SAMPLE_RATE_HZ.hz(), clocks);

        timer.listen(Event::TimeOut);

        // let circle1 = Circle::new(Point::new(128, 64), 64)
        //     .into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
        // let circle2 = Circle::new(Point::new(64, 64), 64)
        //     .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1));

        // let blue_with_red_outline = PrimitiveStyleBuilder::new()
        //     .fill_color(Rgb565::BLUE)
        //     .stroke_color(Rgb565::RED)
        //     .stroke_width(1) // > 1 is not currently supported in embedded-graphics on triangles
        //     .build();
        // let triangle = Triangle::new(
        //     Point::new(40, 120),
        //     Point::new(40, 220),
        //     Point::new(140, 120),
        // )
        // .into_styled(blue_with_red_outline);

        // // draw two circles on black background
        // display.clear(Rgb565::BLACK).unwrap();
        // circle1.draw(&mut display).unwrap();
        // circle2.draw(&mut display).unwrap();
        // triangle.draw(&mut display).unwrap();

        // Text::new("WXYZ", Point::new(10, 10))
        //     .into_styled(TextStyle::new(Font12x16, RUST))
        //     .draw(&mut display)
        //     .unwrap();

        hprintln!("Init complete");
        status.toggle();

        // Init the static resources to use them later through RTIC
        init::LateResources {
            timer,
            display,
            status,
            imu,
            madgwick,
        }
    }

    #[task(binds = TIM2, resources = [timer, display, count, status, madgwick, imu])]
    fn update(cx: update::Context) {
        use core::fmt::Write;
        use heapless::consts::U32;

        let update::Resources {
            timer,
            display,
            count,
            status,
            madgwick,
            imu,
            ..
        } = cx.resources;

        status.toggle();

        let Converted { accel, gyro, mag } =
            convert_measurements(imu.all().expect("IMU read failed"));

        let quat = madgwick.update(accel, gyro, mag);

        // let w = scale(quat.0);
        // let x = scale(quat.1);
        // let y = scale(quat.2);
        // let z = scale(quat.3);

        let vector = F32x3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };

        let rotated = rotate(vector, quat);

        let mut buf = heapless::String::<U32>::new();

        write!(
            &mut buf,
            "{:.2}, {:.2}, {:.2}",
            rotated.x, rotated.y, rotated.z
        );

        let x = (rotated.x * 50.0) as i32;
        let y = (rotated.y * 50.0) as i32;

        Text::new(&buf, Point::new(10, 10))
            .into_styled(
                TextStyleBuilder::new(Font12x16)
                    .text_color(RUST)
                    .background_color(Rgb565::BLACK)
                    .build(),
            )
            .draw(display)
            .unwrap();

        Circle::new(Point::new(120 + x, 120 + y), 10)
            .into_styled(PrimitiveStyle::with_stroke(RUST, 1))
            .draw(display)
            .unwrap();

        // let circle_radius = 5i32;
        // let offs = 10;
        // let top_offs = 40 - offs - 16;

        // // Clear circles
        // Rectangle::new(Point::new(offs, 40), Point::new(64, 240))
        //     .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        //     .draw(display)
        //     .unwrap();

        // Circle::new(
        //     Point::new(offs + circle_radius, top_offs + w),
        //     circle_radius as u32,
        // )
        // .into_styled(PrimitiveStyle::with_stroke(RUST, 1))
        // .draw(display)
        // .unwrap();

        // Circle::new(
        //     Point::new(offs * 2 + circle_radius, top_offs + x),
        //     circle_radius as u32,
        // )
        // .into_styled(PrimitiveStyle::with_stroke(RUST, 1))
        // .draw(display)
        // .unwrap();

        // Circle::new(
        //     Point::new(offs * 3 + circle_radius, top_offs + y),
        //     circle_radius as u32,
        // )
        // .into_styled(PrimitiveStyle::with_stroke(RUST, 1))
        // .draw(display)
        // .unwrap();

        // Circle::new(
        //     Point::new(offs * 4 + circle_radius, top_offs + z),
        //     circle_radius as u32,
        // )
        // .into_styled(PrimitiveStyle::with_stroke(RUST, 1))
        // .draw(display)
        // .unwrap();

        *count += 1;

        timer.clear_interrupt(Event::TimeOut);
    }

    extern "C" {
        fn EXTI0();
    }
};

/// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/derivations/vectors/index.htm
fn rotate(vector: F32x3, quat: Quaternion) -> F32x3 {
    let Quaternion(qw, qx, qy, qz) = quat;
    let F32x3 { x, y, z } = vector;

    let out_x = x * (qx * qx + qw * qw - qy * qy - qz * qz)
        + y * (2.0 * qx * qy - 2.0 * qw * qz)
        + z * (2.0 * qx * qz + 2.0 * qw * qy);
    let out_y = x * (2.0 * qw * qz + 2.0 * qx * qy)
        + y * (qw * qw - qx * qx + qy * qy - qz * qz)
        + z * (-2.0 * qw * qx + 2.0 * qy * qz);
    let out_z = x * (-2.0 * qw * qy + 2.0 * qx * qz)
        + y * (2.0 * qw * qx + 2.0 * qy * qz)
        + z * (qw * qw - qx * qx - qy * qy + qz * qz);

    F32x3 {
        x: out_x,
        y: out_y,
        z: out_z,
    }
}

fn scale(input: f32) -> i32 {
    // Scale -1.0 - 1.0 to 0.0 - 2.0
    let input = input + 1.0;

    // 200px full scale
    let input = input * 100.0;

    input as i32
}

struct Converted {
    accel: F32x3,
    gyro: F32x3,
    mag: F32x3,
}

fn convert_measurements(all: MargMeasurements<[f32; 3]>) -> Converted {
    let MargMeasurements {
        accel, gyro, mag, ..
    } = all;

    Converted {
        accel: F32x3 {
            x: accel[0],
            y: accel[1],
            z: accel[2],
        },
        gyro: F32x3 {
            x: gyro[0],
            y: gyro[1],
            z: gyro[2],
        },
        mag: F32x3 {
            x: mag[0],
            y: mag[1],
            z: mag[2],
        },
    }
}
