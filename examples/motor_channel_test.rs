//! Motor Channel Test - Test all motors simultaneously
//!
//! Tests all 4 motors (M1-M4) by running them forward, stopping, then backward

#![no_std]
#![no_main]

#[allow(dead_code)]
const BUILD_ID: &str = env!("BUILD_ID");

use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Timer};

#[cfg(feature = "usb_serial")]
use embassy_rp::usb::{Driver, InterruptHandler};
#[cfg(feature = "usb_serial")]
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
#[cfg(feature = "usb_serial")]
use embassy_usb::{Builder, Config};

#[cfg(feature = "usb_serial")]
use panic_halt as _;

#[cfg(not(feature = "usb_serial"))]
use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "pico2_w")]
use embedded_alloc::LlffHeap as Heap;

#[cfg(feature = "pico2_w")]
use pico_trail::libraries::motor_driver::Motor;

#[cfg(feature = "pico2_w")]
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[cfg(feature = "pico2_w")]
const HEAP_SIZE: usize = 8 * 1024;

#[cfg(feature = "usb_serial")]
hal::bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<hal::peripherals::USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "pico2_w")]
    {
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        unsafe {
            HEAP.init(
                core::ptr::addr_of_mut!(HEAP_MEM) as *mut u8 as usize,
                HEAP_SIZE,
            )
        }
    }

    #[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
    {
        use embassy_rp::pwm::{Config as PwmConfig, Pwm};

        let peripherals = hal::init(Default::default());

        // Initialize USB Serial
        let driver = Driver::new(peripherals.USB, Irqs);

        let mut usb_config = Config::new(0x2e8a, 0x000a);
        usb_config.manufacturer = Some("Raspberry Pi");
        usb_config.product = Some("Pico 2W Channel Test");
        usb_config.serial_number = Some("12345678");
        usb_config.max_power = 100;
        usb_config.max_packet_size_0 = 64;

        static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
        static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
        static mut CONTROL_BUF: [u8; 64] = [0; 64];
        static mut MSOS_DESCRIPTOR: [u8; 256] = [0; 256];
        static mut STATE: State = State::new();

        let mut builder = Builder::new(
            driver,
            usb_config,
            unsafe { &mut CONFIG_DESCRIPTOR },
            unsafe { &mut BOS_DESCRIPTOR },
            unsafe { &mut MSOS_DESCRIPTOR },
            unsafe { &mut CONTROL_BUF },
        );

        let cdc_class = CdcAcmClass::new(&mut builder, unsafe { &mut STATE }, 64);
        let mut usb_device = builder.build();
        let usb_fut = usb_device.run();

        spawner.spawn(pico_trail::core::logging::usb_logger_task(cdc_class).unwrap());

        let test_fut = async {
            Timer::after(Duration::from_secs(2)).await;

            pico_trail::log_info!("Motor Channel Test [BUILD_ID: {}]", BUILD_ID);
            pico_trail::log_info!("Initializing all 4 motors for simultaneous testing");

            let mut pwm_config = PwmConfig::default();
            pwm_config.top = 999; // 500 Hz @ 125 MHz (matches Freenove)
            pwm_config.divider = 250.into();

            // Initialize all 4 motors with correct PWM slice mappings
            // PWM Slice = GPIO / 2
            pico_trail::log_info!("Initializing M1 (GPIO18/19)");
            let pwm1 = Pwm::new_output_ab(
                peripherals.PWM_SLICE1, // GPIO18/19 -> Slice 1
                peripherals.PIN_18,
                peripherals.PIN_19,
                pwm_config.clone(),
            );
            // M1 needs reversed IN1/IN2 for correct rotation direction
            let (output_a, output_b) = pwm1.split();
            let in2 = pico_trail::platform::rp2350::EmbassyPwmPin {
                output: core::cell::RefCell::new(output_a.expect("PWM channel A")),
            };
            let in1 = pico_trail::platform::rp2350::EmbassyPwmPin {
                output: core::cell::RefCell::new(output_b.expect("PWM channel B")),
            };
            let mut motor1 = pico_trail::libraries::motor_driver::HBridgeMotor::new(in1, in2);

            pico_trail::log_info!("Initializing M2 (GPIO20/21)");
            let pwm2 = Pwm::new_output_ab(
                peripherals.PWM_SLICE2, // GPIO20/21 -> Slice 2
                peripherals.PIN_20,
                peripherals.PIN_21,
                pwm_config.clone(),
            );
            let mut motor2 = pico_trail::platform::rp2350::init_motor_embassy(pwm2);

            pico_trail::log_info!("Initializing M3 (GPIO6/7)");
            let pwm3 = Pwm::new_output_ab(
                peripherals.PWM_SLICE3, // GPIO6/7 -> Slice 3
                peripherals.PIN_6,
                peripherals.PIN_7,
                pwm_config.clone(),
            );
            let mut motor3 = pico_trail::platform::rp2350::init_motor_embassy(pwm3);

            pico_trail::log_info!("Initializing M4 (GPIO8/9)");
            let pwm4 = Pwm::new_output_ab(
                peripherals.PWM_SLICE4, // GPIO8/9 -> Slice 4
                peripherals.PIN_8,
                peripherals.PIN_9,
                pwm_config,
            );
            let mut motor4 = pico_trail::platform::rp2350::init_motor_embassy(pwm4);

            pico_trail::log_info!("All motors initialized, testing all simultaneously");

            // Access internal PWM pins via motor driver
            // We'll use the motor driver's set_speed which controls IN1/IN2

            loop {
                // Test each motor individually first
                pico_trail::log_info!("=== Testing M1 individually ===");
                pico_trail::log_info!("M1 FORWARD");
                if let Err(e) = motor1.set_speed(0.5) {
                    pico_trail::log_error!("M1 forward failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M1 STOP");
                motor1.stop().ok();
                Timer::after(Duration::from_secs(1)).await;

                pico_trail::log_info!("=== Testing M2 individually ===");
                pico_trail::log_info!("M2 FORWARD");
                if let Err(e) = motor2.set_speed(0.5) {
                    pico_trail::log_error!("M2 forward failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M2 STOP");
                motor2.stop().ok();
                Timer::after(Duration::from_secs(1)).await;

                pico_trail::log_info!("=== Testing M3 individually ===");
                pico_trail::log_info!("M3 FORWARD");
                if let Err(e) = motor3.set_speed(0.5) {
                    pico_trail::log_error!("M3 forward failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M3 STOP");
                motor3.stop().ok();
                Timer::after(Duration::from_secs(1)).await;

                pico_trail::log_info!("=== Testing M4 individually ===");
                pico_trail::log_info!("M4 FORWARD");
                if let Err(e) = motor4.set_speed(0.5) {
                    pico_trail::log_error!("M4 forward failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M4 STOP");
                motor4.stop().ok();
                Timer::after(Duration::from_secs(1)).await;

                // Forward - all motors at 50% for 3 seconds
                pico_trail::log_info!("=== ALL MOTORS FORWARD at 50% ===");
                if let Err(e) = motor1.set_speed(0.5) {
                    pico_trail::log_error!("M1 forward failed: {:?}", e);
                }
                if let Err(e) = motor2.set_speed(0.5) {
                    pico_trail::log_error!("M2 forward failed: {:?}", e);
                }
                if let Err(e) = motor3.set_speed(0.5) {
                    pico_trail::log_error!("M3 forward failed: {:?}", e);
                }
                if let Err(e) = motor4.set_speed(0.5) {
                    pico_trail::log_error!("M4 forward failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(3)).await;

                // Wait - 3 second pause
                pico_trail::log_info!("STOP - 3 second pause");
                if let Err(e) = motor1.stop() {
                    pico_trail::log_error!("M1 stop failed: {:?}", e);
                }
                if let Err(e) = motor2.stop() {
                    pico_trail::log_error!("M2 stop failed: {:?}", e);
                }
                if let Err(e) = motor3.stop() {
                    pico_trail::log_error!("M3 stop failed: {:?}", e);
                }
                if let Err(e) = motor4.stop() {
                    pico_trail::log_error!("M4 stop failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(3)).await;

                // Backward - all motors at -50% for 3 seconds
                pico_trail::log_info!("ALL MOTORS BACKWARD at 50%");
                if let Err(e) = motor1.set_speed(-0.5) {
                    pico_trail::log_error!("M1 backward failed: {:?}", e);
                }
                if let Err(e) = motor2.set_speed(-0.5) {
                    pico_trail::log_error!("M2 backward failed: {:?}", e);
                }
                if let Err(e) = motor3.set_speed(-0.5) {
                    pico_trail::log_error!("M3 backward failed: {:?}", e);
                }
                if let Err(e) = motor4.set_speed(-0.5) {
                    pico_trail::log_error!("M4 backward failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(3)).await;

                // Stop and wait before next cycle
                pico_trail::log_info!("STOP - cycle complete");
                if let Err(e) = motor1.stop() {
                    pico_trail::log_error!("M1 stop failed: {:?}", e);
                }
                if let Err(e) = motor2.stop() {
                    pico_trail::log_error!("M2 stop failed: {:?}", e);
                }
                if let Err(e) = motor3.stop() {
                    pico_trail::log_error!("M3 stop failed: {:?}", e);
                }
                if let Err(e) = motor4.stop() {
                    pico_trail::log_error!("M4 stop failed: {:?}", e);
                }
                Timer::after(Duration::from_secs(3)).await;

                pico_trail::log_info!("=== Cycle complete, waiting 5 seconds ===");
                Timer::after(Duration::from_secs(5)).await;
            }
        };

        embassy_futures::join::join(usb_fut, test_fut).await;
    }

    #[cfg(not(feature = "pico2_w"))]
    {
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
