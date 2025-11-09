//! Motor Test - Single Motor Forward/Backward
//!
//! Tests a single motor using Motor trait with proper H-bridge control.
//! Supports both probe-rs (defmt) and USB Serial logging.

#![no_std]
#![no_main]

// Build ID for debugging flush issues (changes on every build)
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

    // USB Serial version
    #[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
    {
        use hal::pwm::{Config as PwmConfig, Pwm};

        let peripherals = hal::init(Default::default());

        // Initialize USB Serial
        let driver = Driver::new(peripherals.USB, Irqs);

        let mut usb_config = Config::new(0x2e8a, 0x000a);
        usb_config.manufacturer = Some("Raspberry Pi");
        usb_config.product = Some("Pico 2W Motor Test");
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

        // Spawn USB device task
        let usb_fut = usb_device.run();

        // Spawn USB logger task
        spawner.spawn(pico_trail::core::logging::usb_logger_task(cdc_class).unwrap());

        // Motor test task
        let motor_test_fut = async {
            // Wait for USB to initialize
            Timer::after(Duration::from_secs(5)).await;

            pico_trail::log_info!("Motor Test - All 4 Motors Sequential Test");
            pico_trail::log_info!("============================================");

            let mut pwm_config = PwmConfig::default();
            pwm_config.top = 999; // 500 Hz @ 125 MHz (matches Freenove)
            pwm_config.divider = 250.into();

            // Initialize all 4 motors with correct PWM slice mappings
            // PWM Slice = GPIO / 2
            pico_trail::log_info!("Initializing all motors...");

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
            pico_trail::log_info!("Motor 1 (GPIO18/19) initialized");

            let pwm2 = Pwm::new_output_ab(
                peripherals.PWM_SLICE2, // GPIO20/21 -> Slice 2
                peripherals.PIN_20,
                peripherals.PIN_21,
                pwm_config.clone(),
            );
            let mut motor2 = pico_trail::platform::rp2350::init_motor_embassy(pwm2);
            pico_trail::log_info!("Motor 2 (GPIO20/21) initialized");

            let pwm3 = Pwm::new_output_ab(
                peripherals.PWM_SLICE3, // GPIO6/7 -> Slice 3
                peripherals.PIN_6,
                peripherals.PIN_7,
                pwm_config.clone(),
            );
            let mut motor3 = pico_trail::platform::rp2350::init_motor_embassy(pwm3);
            pico_trail::log_info!("Motor 3 (GPIO6/7) initialized");

            let pwm4 = Pwm::new_output_ab(
                peripherals.PWM_SLICE4, // GPIO8/9 -> Slice 4
                peripherals.PIN_8,
                peripherals.PIN_9,
                pwm_config,
            );
            let mut motor4 = pico_trail::platform::rp2350::init_motor_embassy(pwm4);
            pico_trail::log_info!("Motor 4 (GPIO8/9) initialized");

            pico_trail::log_info!("PWM Config: top=999, divider=250.0 (500Hz)");
            pico_trail::log_info!("Starting test: Each motor will spin forward 2s -> backward 2s");
            Timer::after(Duration::from_secs(2)).await;

            loop {
                pico_trail::log_info!("=== Cycle Start [BUILD_ID: {}] ===", BUILD_ID);

                // Test Motor 1
                pico_trail::log_info!(">>> Testing Motor 1 (Left Front) <<<");
                pico_trail::log_info!("M1 Forward");
                motor1.set_speed(0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M1 Backward");
                motor1.set_speed(-0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                motor1.stop().ok();
                Timer::after(Duration::from_secs(1)).await;

                // Test Motor 2
                pico_trail::log_info!(">>> Testing Motor 2 (Left Rear) <<<");
                pico_trail::log_info!("M2 Forward");
                motor2.set_speed(0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M2 Backward");
                motor2.set_speed(-0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                motor2.stop().ok();
                Timer::after(Duration::from_secs(1)).await;

                // Test Motor 3
                pico_trail::log_info!(">>> Testing Motor 3 (Right Rear) <<<"); // GPIO6/7
                pico_trail::log_info!("M3 Forward");
                motor3.set_speed(0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M3 Backward");
                motor3.set_speed(-0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                motor3.stop().ok();
                Timer::after(Duration::from_secs(1)).await;

                // Test Motor 4
                pico_trail::log_info!(">>> Testing Motor 4 (Right Front) <<<"); // GPIO8/9
                pico_trail::log_info!("M4 Forward");
                motor4.set_speed(0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                pico_trail::log_info!("M4 Backward");
                motor4.set_speed(-0.5).ok();
                Timer::after(Duration::from_secs(2)).await;
                motor4.stop().ok();
                Timer::after(Duration::from_secs(3)).await;

                pico_trail::log_info!("=== All motors tested, repeating ===");
            }
        };

        embassy_futures::join::join(usb_fut, motor_test_fut).await;
    }

    // probe-rs version (defmt-rtt)
    #[cfg(all(feature = "pico2_w", not(feature = "usb_serial")))]
    {
        use hal::pwm::{Config as PwmConfig, Pwm};

        let peripherals = hal::init(Default::default());

        pico_trail::log_info!("Motor Test - Single Motor Forward/Backward");
        pico_trail::log_info!("===========================================");

        let mut pwm_config = PwmConfig::default();
        pwm_config.top = 999; // 500 Hz @ 125 MHz (matches Freenove)
        pwm_config.divider = 250.into();

        // Initialize Motor 1 using init_motor_embassy
        // PWM Slice = GPIO / 2
        pico_trail::log_info!("Initializing Motor 1 (GPIO18/19)...");
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

        pico_trail::log_info!("Motor initialized successfully");

        loop {
            pico_trail::log_info!("=== Cycle Start [BUILD_ID: {}] ===", BUILD_ID);

            // Forward: 50% speed
            pico_trail::log_info!("FORWARD at 50% speed");
            if let Err(e) = motor1.set_speed(0.5) {
                pico_trail::log_error!("Failed to set forward speed: {:?}", e);
            }
            Timer::after(Duration::from_secs(3)).await;

            // Stop
            pico_trail::log_info!("STOP");
            if let Err(e) = motor1.stop() {
                pico_trail::log_error!("Failed to stop: {:?}", e);
            }
            Timer::after(Duration::from_secs(1)).await;

            // Backward: -50% speed
            pico_trail::log_info!("BACKWARD at 50% speed");
            if let Err(e) = motor1.set_speed(-0.5) {
                pico_trail::log_error!("Failed to set backward speed: {:?}", e);
            }
            Timer::after(Duration::from_secs(3)).await;

            // Stop
            pico_trail::log_info!("STOP");
            if let Err(e) = motor1.stop() {
                pico_trail::log_error!("Failed to stop: {:?}", e);
            }
            Timer::after(Duration::from_secs(2)).await;
        }
    }

    #[cfg(not(feature = "pico2_w"))]
    {
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
