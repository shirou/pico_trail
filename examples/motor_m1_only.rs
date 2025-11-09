//! Motor M1 Only Test - Minimal test for M1 motor
//!
//! Tests only M1 (GPIO18/19) to diagnose initialization issues

#![no_std]
#![no_main]

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
        usb_config.product = Some("Pico 2W M1 Test");
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
            Timer::after(Duration::from_secs(5)).await;

            pico_trail::log_info!("=== M1 ONLY TEST ===");
            pico_trail::log_info!("Testing GPIO18/19 with PWM_SLICE1");

            let mut pwm_config = PwmConfig::default();
            pwm_config.top = 999; // 500 Hz @ 125 MHz (matches Freenove)
            pwm_config.divider = 250.into();

            // Initialize ALL motors (like motor_no_split_test)
            pico_trail::log_info!("Initializing all 4 motors...");

            pico_trail::log_info!("Creating PWM for M1 (GPIO18/19)...");
            let pwm1 = Pwm::new_output_ab(
                peripherals.PWM_SLICE1,
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
            pico_trail::log_info!("M1 initialized");

            pico_trail::log_info!("Creating PWM for M2 (GPIO20/21)...");
            let pwm2 = Pwm::new_output_ab(
                peripherals.PWM_SLICE2,
                peripherals.PIN_20,
                peripherals.PIN_21,
                pwm_config.clone(),
            );
            let _motor2 = pico_trail::platform::rp2350::init_motor_embassy(pwm2);
            pico_trail::log_info!("M2 initialized");

            pico_trail::log_info!("Creating PWM for M3 (GPIO6/7)...");
            let pwm3 = Pwm::new_output_ab(
                peripherals.PWM_SLICE3,
                peripherals.PIN_6,
                peripherals.PIN_7,
                pwm_config.clone(),
            );
            let _motor3 = pico_trail::platform::rp2350::init_motor_embassy(pwm3);
            pico_trail::log_info!("M3 initialized");

            pico_trail::log_info!("Creating PWM for M4 (GPIO8/9)...");
            let pwm4 = Pwm::new_output_ab(
                peripherals.PWM_SLICE4,
                peripherals.PIN_8,
                peripherals.PIN_9,
                pwm_config,
            );
            let _motor4 = pico_trail::platform::rp2350::init_motor_embassy(pwm4);
            pico_trail::log_info!("M4 initialized");

            pico_trail::log_info!("All motors initialized, testing M1 only");

            loop {
                pico_trail::log_info!("=== M1 FORWARD ===");
                if let Err(e) = motor1.set_speed(0.5) {
                    pico_trail::log_error!("M1 forward failed: {:?}", e);
                } else {
                    pico_trail::log_info!("M1 forward command sent");
                }
                Timer::after(Duration::from_secs(3)).await;

                pico_trail::log_info!("=== M1 STOP ===");
                if let Err(e) = motor1.stop() {
                    pico_trail::log_error!("M1 stop failed: {:?}", e);
                } else {
                    pico_trail::log_info!("M1 stop command sent");
                }
                Timer::after(Duration::from_secs(2)).await;

                pico_trail::log_info!("=== M1 BACKWARD ===");
                if let Err(e) = motor1.set_speed(-0.5) {
                    pico_trail::log_error!("M1 backward failed: {:?}", e);
                } else {
                    pico_trail::log_info!("M1 backward command sent");
                }
                Timer::after(Duration::from_secs(3)).await;

                pico_trail::log_info!("=== M1 STOP ===");
                motor1.stop().ok();
                Timer::after(Duration::from_secs(2)).await;
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
