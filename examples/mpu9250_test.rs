//! MPU-9250 IMU Test
//!
//! Simple test to read accelerometer/gyroscope data from MPU-9250.
//! Outputs data to USB Serial.
//!
//! # Hardware
//! - MPU-9250 I2C address: 0x68
//! - SDA: GPIO 4, SCL: GPIO 5
//!
//! # Usage
//! ```bash
//! EXTRA_FEATURES="usb_serial" ./scripts/build-rp2350.sh mpu9250_test
//! screen /dev/ttyACM0 115200
//! ```

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::I2c;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use panic_halt as _;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

const HEAP_SIZE: usize = 8 * 1024;

// MPU-9250 I2C address
const MPU9250_ADDR: u8 = 0x68;

// MPU-9250 registers
const REG_WHO_AM_I: u8 = 0x75;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_GYRO_XOUT_H: u8 = 0x43;

// Expected WHO_AM_I values
const WHO_AM_I_MPU9250: u8 = 0x71;
const WHO_AM_I_MPU9255: u8 = 0x73;
const WHO_AM_I_MPU6500: u8 = 0x70;

hal::bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<hal::peripherals::USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<hal::peripherals::I2C0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize heap
    {
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        unsafe {
            HEAP.init(addr_of_mut!(HEAP_MEM) as *mut u8 as usize, HEAP_SIZE);
        }
    }

    let p = hal::init(Default::default());

    // Onboard LED
    let mut led = Output::new(p.PIN_25, Level::Low);

    // Blink to show boot
    for _ in 0..3 {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(100)).await;
    }

    // Initialize USB Serial
    let driver = Driver::new(p.USB, Irqs);

    let mut usb_config = Config::new(0x2e8a, 0x000a);
    usb_config.manufacturer = Some("Raspberry Pi");
    usb_config.product = Some("MPU9250 Test");
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
        unsafe { &mut *addr_of_mut!(CONFIG_DESCRIPTOR) },
        unsafe { &mut *addr_of_mut!(BOS_DESCRIPTOR) },
        unsafe { &mut *addr_of_mut!(MSOS_DESCRIPTOR) },
        unsafe { &mut *addr_of_mut!(CONTROL_BUF) },
    );

    let mut cdc = CdcAcmClass::new(&mut builder, unsafe { &mut *addr_of_mut!(STATE) }, 64);
    let usb = builder.build();

    spawner.spawn(usb_task(usb).unwrap());

    // Wait for USB
    Timer::after(Duration::from_secs(2)).await;

    let _ = cdc.write_packet(b"\r\n=== MPU-9250 Test ===\r\n").await;
    Timer::after(Duration::from_millis(50)).await;

    // Initialize I2C0 (GPIO 4 = SDA, GPIO 5 = SCL)
    let mut i2c = I2c::new_async(
        p.I2C0,
        p.PIN_5, // SCL
        p.PIN_4, // SDA
        Irqs,
        {
            let mut config = embassy_rp::i2c::Config::default();
            config.frequency = 400_000;
            config
        },
    );

    let _ = cdc.write_packet(b"I2C initialized\r\n").await;

    // Read WHO_AM_I register
    let mut who_am_i = [0u8; 1];
    match i2c
        .write_read(MPU9250_ADDR, &[REG_WHO_AM_I], &mut who_am_i)
        .await
    {
        Ok(_) => {
            let id = who_am_i[0];
            let name = match id {
                WHO_AM_I_MPU9250 => "MPU-9250",
                WHO_AM_I_MPU9255 => "MPU-9255",
                WHO_AM_I_MPU6500 => "MPU-6500",
                _ => "Unknown MPU",
            };

            let mut msg = [0u8; 48];
            let len = format_msg(&mut msg, b"WHO_AM_I: 0x", id, name);
            let _ = cdc.write_packet(&msg[..len]).await;
        }
        Err(_) => {
            let _ = cdc
                .write_packet(b"ERROR: Failed to read WHO_AM_I\r\n")
                .await;
        }
    }
    Timer::after(Duration::from_millis(50)).await;

    // Wake up MPU-9250 (clear sleep bit)
    if i2c
        .write(MPU9250_ADDR, &[REG_PWR_MGMT_1, 0x00])
        .await
        .is_ok()
    {
        let _ = cdc.write_packet(b"MPU-9250 initialized\r\n").await;
    } else {
        let _ = cdc.write_packet(b"ERROR: Failed to initialize\r\n").await;
    }

    Timer::after(Duration::from_millis(100)).await;

    let _ = cdc.write_packet(b"\r\nReading sensor data...\r\n").await;
    Timer::after(Duration::from_millis(50)).await;

    // Main loop - read accelerometer and gyroscope
    let mut count = 0u32;
    loop {
        // Read accelerometer (6 bytes: XH,XL,YH,YL,ZH,ZL)
        let mut accel_data = [0u8; 6];
        let mut gyro_data = [0u8; 6];

        let accel_ok = i2c
            .write_read(MPU9250_ADDR, &[REG_ACCEL_XOUT_H], &mut accel_data)
            .await
            .is_ok();
        let gyro_ok = i2c
            .write_read(MPU9250_ADDR, &[REG_GYRO_XOUT_H], &mut gyro_data)
            .await
            .is_ok();

        if accel_ok && gyro_ok {
            // Parse raw values (big-endian, signed 16-bit)
            let ax = i16::from_be_bytes([accel_data[0], accel_data[1]]);
            let ay = i16::from_be_bytes([accel_data[2], accel_data[3]]);
            let az = i16::from_be_bytes([accel_data[4], accel_data[5]]);

            let gx = i16::from_be_bytes([gyro_data[0], gyro_data[1]]);
            let gy = i16::from_be_bytes([gyro_data[2], gyro_data[3]]);
            let gz = i16::from_be_bytes([gyro_data[4], gyro_data[5]]);

            // Print every 10th reading
            count += 1;
            if count % 10 == 0 {
                let mut buf = [0u8; 64];
                let len = format_imu_data(&mut buf, ax, ay, az, gx, gy, gz);
                let _ = cdc.write_packet(&buf[..len]).await;
            }

            led.toggle();
        }

        Timer::after(Duration::from_millis(20)).await; // 50Hz
    }
}

#[embassy_executor::task]
async fn usb_task(
    mut usb: embassy_usb::UsbDevice<'static, Driver<'static, hal::peripherals::USB>>,
) {
    usb.run().await;
}

fn format_msg(buf: &mut [u8], prefix: &[u8], val: u8, suffix: &str) -> usize {
    const HEX: &[u8] = b"0123456789ABCDEF";
    let mut pos = 0;

    buf[pos..pos + prefix.len()].copy_from_slice(prefix);
    pos += prefix.len();

    buf[pos] = HEX[(val >> 4) as usize];
    buf[pos + 1] = HEX[(val & 0x0F) as usize];
    pos += 2;

    buf[pos] = b' ';
    buf[pos + 1] = b'(';
    pos += 2;

    let suffix_bytes = suffix.as_bytes();
    buf[pos..pos + suffix_bytes.len()].copy_from_slice(suffix_bytes);
    pos += suffix_bytes.len();

    buf[pos..pos + 3].copy_from_slice(b")\r\n");
    pos += 3;

    pos
}

fn format_imu_data(buf: &mut [u8], ax: i16, ay: i16, az: i16, gx: i16, gy: i16, gz: i16) -> usize {
    // Simple format: "A:x,y,z G:x,y,z\r\n"
    let mut pos = 0;

    buf[pos..pos + 2].copy_from_slice(b"A:");
    pos += 2;
    pos += format_i16(&mut buf[pos..], ax);
    buf[pos] = b',';
    pos += 1;
    pos += format_i16(&mut buf[pos..], ay);
    buf[pos] = b',';
    pos += 1;
    pos += format_i16(&mut buf[pos..], az);

    buf[pos..pos + 3].copy_from_slice(b" G:");
    pos += 3;
    pos += format_i16(&mut buf[pos..], gx);
    buf[pos] = b',';
    pos += 1;
    pos += format_i16(&mut buf[pos..], gy);
    buf[pos] = b',';
    pos += 1;
    pos += format_i16(&mut buf[pos..], gz);

    buf[pos..pos + 2].copy_from_slice(b"\r\n");
    pos += 2;

    pos
}

fn format_i16(buf: &mut [u8], val: i16) -> usize {
    let mut pos = 0;
    let mut v = val;

    if v < 0 {
        buf[pos] = b'-';
        pos += 1;
        v = -v;
    }

    let mut digits = [0u8; 6];
    let mut len = 0;
    let mut n = v as u16;

    if n == 0 {
        buf[pos] = b'0';
        return pos + 1;
    }

    while n > 0 {
        digits[len] = b'0' + (n % 10) as u8;
        n /= 10;
        len += 1;
    }

    for i in (0..len).rev() {
        buf[pos] = digits[i];
        pos += 1;
    }

    pos
}
