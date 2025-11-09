//! Motor M4 Only Test - Test for M4 motor (known to work)
//!
//! Tests only M4 (GPIO8/9) to compare with M1

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
        usb_config.product = Some("Pico 2W M4 Test");
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

            pico_trail::log_info!("=== M4 ONLY TEST ===");
            pico_trail::log_info!("Testing GPIO8/9 with PWM_SLICE4");

            let mut pwm_config = PwmConfig::default();
            pwm_config.top = 999; // 500 Hz @ 125 MHz (matches Freenove)
            pwm_config.divider = 250.into();

            pico_trail::log_info!("Creating PWM for M4 (GPIO8/9)...");
            let pwm4 = Pwm::new_output_ab(
                peripherals.PWM_SLICE4,
                peripherals.PIN_8,
                peripherals.PIN_9,
                pwm_config,
            );
            pico_trail::log_info!("PWM created successfully");

            pico_trail::log_info!("Initializing motor driver...");
            let mut motor4 = pico_trail::platform::rp2350::init_motor_embassy(pwm4);
            pico_trail::log_info!("Motor initialized successfully");

            loop {
                pico_trail::log_info!("=== M4 FORWARD ===");
                pico_trail::log_info!("Motor set_speed: 0.5");
                if let Err(e) = motor4.set_speed(0.5) {
                    pico_trail::log_error!("M4 forward failed: {:?}", e);
                } else {
                    pico_trail::log_info!("M4 forward command sent");
                }
                Timer::after(Duration::from_secs(3)).await;

                pico_trail::log_info!("=== M4 STOP ===");
                if let Err(e) = motor4.stop() {
                    pico_trail::log_error!("M4 stop failed: {:?}", e);
                } else {
                    pico_trail::log_info!("M4 stop command sent");
                }
                Timer::after(Duration::from_secs(2)).await;

                pico_trail::log_info!("=== M4 BACKWARD ===");
                pico_trail::log_info!("Motor set_speed: -0.5");
                if let Err(e) = motor4.set_speed(-0.5) {
                    pico_trail::log_error!("M4 backward failed: {:?}", e);
                } else {
                    pico_trail::log_info!("M4 backward command sent");
                }
                Timer::after(Duration::from_secs(3)).await;

                pico_trail::log_info!("=== M4 STOP ===");
                motor4.stop().ok();
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
