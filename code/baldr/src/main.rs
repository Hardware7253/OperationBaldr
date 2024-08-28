#![no_main]
#![no_std]


use panic_rtt_core::{self, rtt_init_print, rprintln};

use stm32f4xx_hal as hal;

use cortex_m::interrupt::Mutex;
use core::cell::RefCell;
use crate::hal::{
    i2c::{I2c, Mode as ModeI2c},
    gpio,
    interrupt,
    spi::*,
    pac,
    timer,
    pac::DWT,
    prelude::*
};
use cortex_m_rt::entry;
use ds323x::{DateTimeAccess, Ds323x, NaiveDate, Timelike, Rtcc};
use arrform::{arrform, ArrForm};


pub mod vfd;
pub mod cycle_counter;
pub mod button;
pub mod helpers;

use helpers::add_with_rollover;

const CLOCK_MHZ: u32 = 100;

const FILAMENT_TIMER_FREQ: f32 = 100.0; // Frequency to generate ac voltage across the VFD filament pins (Hz)
const FILAMET_TIMER_WAIT_US: u32 = ((1.0 / FILAMENT_TIMER_FREQ) * 1000000.0) as u32;

const POPULATION_GRADIENT: f64 = 79217336.08549047;
const POPULATION_Y_INTERCEPT: f64 = -152274836829.88293;

const UPDATE_POPULATION_MS: u64 = 1000; // How many millseconds inbetween the population screen being updated


static G_FILAMENT_TIMER: Mutex<RefCell<Option<timer::CounterMs<pac::TIM2>>>> = Mutex::new(RefCell::new(None));

// Create a global variables for filament pins
// These need to be global so they can be used in the timer ISR
type FilamentPin1 = gpio::PB2<gpio::Output<gpio::PushPull>>;
type FilamentPin2 = gpio::PB1<gpio::Output<gpio::PushPull>>;
static G_FILAMENT_1: Mutex<RefCell<Option<FilamentPin1>>> = Mutex::new(RefCell::new(None));
static G_FILAMENT_2: Mutex<RefCell<Option<FilamentPin2>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let mut cp = cortex_m::Peripherals::take().unwrap(); // Core peripherals
    let dp = pac::Peripherals::take().unwrap(); // Device peripherals

    // Configure clocks
    let rcc = dp.RCC.constrain(); 
    let clocks = rcc.cfgr
        // External oscillator
        .use_hse(8.MHz())

        // Bus and core clocks
        .hclk(CLOCK_MHZ.MHz())
        .sysclk(CLOCK_MHZ.MHz())

        // Peripheral clocks
        .pclk1(12.MHz())
        .pclk2(12.MHz())
    .freeze();

    // Setup delay
    let mut delay = cp.SYST.delay(&clocks);

    // Enable cycle counter
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // Promote gpio pac structs
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // Setup filament timer
    let mut filament_timer = dp.TIM2.counter_ms(&clocks);
    filament_timer.start(FILAMET_TIMER_WAIT_US.micros()).unwrap();
    filament_timer.listen(timer::Event::Update);

    // Obtain filament pin handles, and intialise them so that there is a potential difference accross the filament wires
    let mut filament_1 = gpiob.pb2.into_push_pull_output();
    let mut filament_2 = gpiob.pb1.into_push_pull_output();
    filament_1.set_high();
    filament_2.set_low();

    // Enable interrupt source in NVIC
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
    }

    // Move local variables to global context
    cortex_m::interrupt::free(|cs| {
        G_FILAMENT_TIMER.borrow(cs).replace(Some(filament_timer));
        G_FILAMENT_1.borrow(cs).replace(Some(filament_1));
        G_FILAMENT_2.borrow(cs).replace(Some(filament_2));
    });

    // Construct rtc using i2c bus
    let scl = gpiob.pb6;
    let sda = gpiob.pb7;

    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        ModeI2c::standard(400.kHz()),
        &clocks,
    );

    let mut rtc = Ds323x::new_ds3231(i2c);

    // Construct vfd using spi bus
    let spi_pins = (
        gpioa.pa5, // Clock
        NoMiso::new(), // Miso
        gpioa.pa7// Mosi
    );

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let mut vfd = vfd::Vfd {
        spi_bus: Spi::new(dp.SPI1, spi_pins, spi_mode, 100.kHz(), &clocks),
        latch_pin: gpioa.pa4.into_push_pull_output(),
        oe_pin: gpioa.pa6.into_open_drain_output(),
    };

    vfd.init(&mut delay);

    // Create a cycle counter struct to accurately keep track of elapsed clock cycles
    let mut cycle_counter = cycle_counter::Counter::new();

    // Setup switch pins as pulldown
    let on_switch_pin = gpioa.pa9.into_pull_down_input();
    let mode_switch_pin = gpioa.pa8.into_pull_down_input();

    // Buttons for adjusting the time
    let mut buttons = (
        button::Button::new(gpioa.pa12.into_pull_down_input()), // Hours
        button::Button::new(gpioa.pa11.into_pull_down_input()), // Minutes
        button::Button::new(gpioa.pa10.into_pull_down_input()), // Seconds
    );

    let mut time_adjust; // True when the users wants to adjust the clock time
    let mut can_exit_time_adjust = false;
    

    // Set rtc datetime
    // let datetime = NaiveDate::from_ymd_opt(2024, 8, 26)
    //     .unwrap()
    //     .and_hms_opt(10, 33, 0)
    //     .unwrap();
    // rtc.set_datetime(&datetime).unwrap();

    let mut update_population_cycles: u64 = 0;
    let mut population_af = arrform!(64, "{:011}", 0);

    loop {
        if on_switch_pin.is_high() { // Only do VFD stuff when the on switch is high
            cycle_counter.update();

            // Read time from rtc
            let time = rtc.time().unwrap();
            let mut hour = time.hour();
            let mut minute = time.minute();
            let mut second = time.second();

            if mode_switch_pin.is_high() {

                // Model the population of humans on earth when the mode switch is active
                // Because we can

                // Update population periodically because it's slow
                if cycle_counter.cycles > update_population_cycles {
                    let year = rtc.year().unwrap() as f64
                    + (rtc.month().unwrap() as f64 / 12.0)
                    + (rtc.day().unwrap() as f64 / 30.0 / 12.0)
                    + (hour as f64 / 24.0 / 30.0 / 12.0)
                    + (minute as f64 / 60.0 / 24.0 / 30.0 / 12.0)
                    + (second as f64 / 60.0 / 60.0 / 24.0 / 30.0 / 12.0);


                    let population = POPULATION_GRADIENT * year + POPULATION_Y_INTERCEPT;
                    population_af = arrform!(64, "{:011}", population as u64);

                    update_population_cycles = cycle_counter.cycles + helpers::ms_to_cycles(UPDATE_POPULATION_MS, CLOCK_MHZ as u64); // Reset timer
                }
                
                vfd.display_str(population_af.as_str(), &mut delay); // Display current population estimate
            } else {

                // This code is run when the device is in clock mode
                cycle_counter.update();

                // Update button structs
                buttons.0.pressed(cycle_counter.cycles);
                buttons.1.pressed(cycle_counter.cycles);
                buttons.2.pressed(cycle_counter.cycles);


                // Prevent time adjust mode being reactivated immedeately after leaving time adjust mode
                // Because the buttons would still all be registering a long press
                if (!buttons.0.press_raw || !buttons.1.press_raw || !buttons.2.press_raw) && can_exit_time_adjust {
                    can_exit_time_adjust = false;
                }

                // If all 3 buttons are held down activate time adjust mode
                time_adjust = buttons.0.long_press && buttons.1.long_press && buttons.2.long_press && !can_exit_time_adjust;



                // Run time adjust mode in a while loop to prevent the rtc from incrementing the hours, minutes, or seconds while the user is trying to adjust.
                while time_adjust {
                    cycle_counter.update();

                    // Update button structs
                    let button_0_pressed = buttons.0.pressed(cycle_counter.cycles);
                    let button_1_pressed = buttons.1.pressed(cycle_counter.cycles);
                    let button_2_pressed = buttons.2.pressed(cycle_counter.cycles);

                    // Once any button is released it becomes possible for the user to exit time adjust mode
                    if (!buttons.0.press_raw || !buttons.1.press_raw || !buttons.2.press_raw) && !can_exit_time_adjust {
                        can_exit_time_adjust = true;
                    }

                    // If the user activates another long press exit time adjust mode
                    if buttons.0.long_press && buttons.1.long_press && buttons.2.long_press && can_exit_time_adjust {
                        time_adjust = false;
                    }

                    // Increment hours minutes and seconds when their respective buttons are pressed
                    if button_0_pressed && !buttons.1.press_raw && !buttons.2.press_raw {
                        hour = add_with_rollover(hour, 1, 0, 23);
                    }

                    if button_1_pressed && !buttons.2.press_raw && !buttons.0.press_raw {
                        minute = add_with_rollover(minute, 1, 0, 59);
                    }

                    if button_2_pressed && !buttons.0.press_raw && !buttons.1.press_raw {
                        second = add_with_rollover(second, 1, 0, 59);
                    }

                    // Update rtc datetime
                    if button_0_pressed || button_1_pressed || button_2_pressed {
                        let datetime = rtc.date()
                        .unwrap()
                        .and_hms_opt(hour, minute, second)
                        .unwrap();
                        rtc.set_datetime(&datetime).unwrap();
                    }

                    // Dsiplay adjusted time
                    let time_af = arrform!(64, "  {}{} {}{} {}{} ", 
                        hour / 10, hour % 10,
                        minute / 10, minute % 10,
                        second / 10, second % 10,
                    );

                    vfd.display_str(time_af.as_str(), &mut delay);
                }

                // Display time
                let time_af = arrform!(64, "  {}{}.{}{}.{}{} ", 
                        hour / 10, hour % 10,
                        minute / 10, minute % 10,
                        second / 10, second % 10,
                );

                vfd.display_str(time_af.as_str(), &mut delay);
            }
        } else {
            vfd.turn_off(&mut delay);
        }
    }
}

// Timer Interrupt
#[interrupt]
fn TIM2() {
    // Start a critical selection inorder to access shared resources
    cortex_m::interrupt::free(|cs| {
        
        // Toggle filament pins
        let mut filament_1 = G_FILAMENT_1.borrow(cs).borrow_mut();
        filament_1.as_mut().unwrap().toggle();

        let mut filament_2 = G_FILAMENT_2.borrow(cs).borrow_mut();
        filament_2.as_mut().unwrap().toggle();

        // Restart timer
        let mut timer = G_FILAMENT_TIMER.borrow(cs).borrow_mut();
        timer.as_mut().unwrap().start(FILAMET_TIMER_WAIT_US.micros()).unwrap();
    });
}