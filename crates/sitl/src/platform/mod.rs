//! Simulated platform for SITL.
//!
//! `SitlPlatform` provides simulated peripherals (UART, PWM, GPIO, Timer)
//! and bridges sensor data from the simulator adapter to the autopilot,
//! while collecting actuator commands for the adapter.

pub mod gpio;
pub mod pwm;
pub mod timer;
pub mod uart;

pub use gpio::{GpioDirection, SitlGpio};
pub use pwm::SitlPwm;
pub use timer::SitlTimeSource;
pub use uart::SitlUart;

use std::sync::Mutex;

use crate::types::{ActuatorCommands, SensorData, VehicleId};

/// Maximum number of UART peripherals.
const MAX_UARTS: u8 = 4;
/// Maximum number of PWM channels.
const MAX_PWMS: usize = 8;
/// Maximum GPIO pin number.
const MAX_GPIO: u8 = 29;

/// Simulated platform providing peripheral access and sensor/actuator bridging.
///
/// The bridge injects sensor data via [`inject_sensors`](SitlPlatform::inject_sensors)
/// and reads actuator commands via [`collect_actuator_commands`](SitlPlatform::collect_actuator_commands).
pub struct SitlPlatform {
    vehicle_id: VehicleId,
    time_source: SitlTimeSource,

    /// Injected sensor data (written by bridge, read by autopilot).
    sensor_data: Mutex<Option<SensorData>>,

    /// PWM channels tracked for actuator collection.
    pwm_channels: Mutex<Vec<SitlPwm>>,

    /// Number of UARTs created.
    uart_count: Mutex<u8>,

    /// Allocated GPIO pins.
    gpio_allocated: Mutex<Vec<u8>>,

    /// Configurable battery ADC value.
    battery_adc_value: Mutex<u16>,
}

impl SitlPlatform {
    /// Create a new SITL platform for the given vehicle.
    pub fn new(vehicle_id: VehicleId) -> Self {
        Self {
            vehicle_id,
            time_source: SitlTimeSource::new(),
            sensor_data: Mutex::new(None),
            pwm_channels: Mutex::new(Vec::new()),
            uart_count: Mutex::new(0),
            gpio_allocated: Mutex::new(Vec::new()),
            battery_adc_value: Mutex::new(0),
        }
    }

    /// Create a new SITL platform with a shared time source.
    pub fn with_time_source(vehicle_id: VehicleId, time_source: SitlTimeSource) -> Self {
        Self {
            vehicle_id,
            time_source,
            sensor_data: Mutex::new(None),
            pwm_channels: Mutex::new(Vec::new()),
            uart_count: Mutex::new(0),
            gpio_allocated: Mutex::new(Vec::new()),
            battery_adc_value: Mutex::new(0),
        }
    }

    /// Get simulated system clock frequency (125 MHz, matching RP2350).
    pub fn system_clock_hz(&self) -> u32 {
        125_000_000
    }

    /// Create a simulated UART peripheral.
    pub fn create_uart(&self, uart_id: u8, baud_rate: u32) -> Result<SitlUart, &'static str> {
        let mut count = self.uart_count.lock().unwrap();
        if uart_id >= MAX_UARTS {
            return Err("UART ID out of range");
        }
        *count += 1;
        Ok(SitlUart::new(baud_rate))
    }

    /// Create a simulated PWM channel.
    ///
    /// The PWM channel is tracked internally so actuator commands can be
    /// collected later via [`collect_actuator_commands`](Self::collect_actuator_commands).
    pub fn create_pwm(&self, pin: u8, frequency: u32) -> Result<usize, &'static str> {
        let mut channels = self.pwm_channels.lock().unwrap();
        if channels.len() >= MAX_PWMS {
            return Err("Maximum PWM channels reached");
        }
        let index = channels.len();
        channels.push(SitlPwm::new(pin, frequency, 0.0));
        Ok(index)
    }

    /// Set the duty cycle of a tracked PWM channel by index.
    pub fn set_pwm_duty(&self, index: usize, duty_cycle: f32) {
        let mut channels = self.pwm_channels.lock().unwrap();
        if let Some(pwm) = channels.get_mut(index) {
            pwm.set_duty_cycle(duty_cycle);
        }
    }

    /// Get the duty cycle of a tracked PWM channel by index.
    pub fn get_pwm_duty(&self, index: usize) -> Option<f32> {
        let channels = self.pwm_channels.lock().unwrap();
        channels.get(index).map(|pwm| pwm.duty_cycle())
    }

    /// Create a simulated GPIO pin.
    pub fn create_gpio(&self, pin: u8) -> Result<SitlGpio, &'static str> {
        let mut allocated = self.gpio_allocated.lock().unwrap();
        if pin > MAX_GPIO {
            return Err("GPIO pin out of range");
        }
        if allocated.contains(&pin) {
            return Err("GPIO pin already allocated");
        }
        allocated.push(pin);
        Ok(SitlGpio::new_output(pin))
    }

    /// Get a reference to the time source.
    pub fn time_source(&self) -> &SitlTimeSource {
        &self.time_source
    }

    /// Read the configurable battery ADC value.
    pub fn read_battery_adc(&self) -> u16 {
        *self.battery_adc_value.lock().unwrap()
    }

    /// Set the battery ADC value for testing.
    pub fn set_battery_adc_value(&self, value: u16) {
        *self.battery_adc_value.lock().unwrap() = value;
    }

    /// Get the vehicle ID.
    pub fn vehicle_id(&self) -> VehicleId {
        self.vehicle_id
    }

    // --- Sensor injection (bridge → platform) ---

    /// Inject sensor data from the bridge/adapter.
    pub fn inject_sensors(&self, data: &SensorData) {
        let mut sensor = self.sensor_data.lock().unwrap();
        *sensor = Some(data.clone());
    }

    /// Take the latest sensor data (returns None if no new data available).
    pub fn take_sensors(&self) -> Option<SensorData> {
        let mut sensor = self.sensor_data.lock().unwrap();
        sensor.take()
    }

    /// Peek at the current sensor data without consuming it.
    ///
    /// Unlike `take_sensors()`, this clones the data so it remains available
    /// for subsequent reads.
    pub fn peek_sensors(&self) -> Option<SensorData> {
        let sensor = self.sensor_data.lock().unwrap();
        sensor.clone()
    }

    // --- Actuator collection (platform → bridge) ---

    /// Collect actuator commands from PWM channel duty cycles.
    ///
    /// Motor outputs are mapped from duty cycle [0.0, 1.0] to normalized
    /// range [-1.0, 1.0] (center at 0.5 maps to 0.0).
    pub fn collect_actuator_commands(&self) -> ActuatorCommands {
        let channels = self.pwm_channels.lock().unwrap();
        let motors: Vec<f32> = channels
            .iter()
            .map(|pwm| {
                // Map duty cycle 0.0..1.0 to -1.0..1.0
                (pwm.duty_cycle() - 0.5) * 2.0
            })
            .collect();

        ActuatorCommands {
            timestamp_us: self.time_source.now_us(),
            vehicle_id: self.vehicle_id,
            motors,
            servos: Vec::new(),
        }
    }
}

impl std::fmt::Debug for SitlPlatform {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SitlPlatform")
            .field("vehicle_id", &self.vehicle_id)
            .field("time_source", &self.time_source)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{GpsData, GpsFixType, ImuData};

    #[test]
    fn test_platform_creation() {
        let platform = SitlPlatform::new(VehicleId(1));
        assert_eq!(platform.vehicle_id(), VehicleId(1));
        assert_eq!(platform.system_clock_hz(), 125_000_000);
    }

    #[test]
    fn test_uart_creation() {
        let platform = SitlPlatform::new(VehicleId(1));
        let uart = platform.create_uart(0, 115200);
        assert!(uart.is_ok());
        assert_eq!(uart.unwrap().baud_rate(), 115200);

        // Invalid UART ID
        assert!(platform.create_uart(10, 115200).is_err());
    }

    #[test]
    fn test_pwm_creation_and_duty() {
        let platform = SitlPlatform::new(VehicleId(1));
        let idx = platform.create_pwm(0, 50).unwrap();
        assert_eq!(platform.get_pwm_duty(idx), Some(0.0));

        platform.set_pwm_duty(idx, 0.75);
        assert_eq!(platform.get_pwm_duty(idx), Some(0.75));
    }

    #[test]
    fn test_gpio_creation() {
        let platform = SitlPlatform::new(VehicleId(1));
        let gpio = platform.create_gpio(5);
        assert!(gpio.is_ok());

        // Same pin should fail
        assert!(platform.create_gpio(5).is_err());

        // Invalid pin
        assert!(platform.create_gpio(100).is_err());
    }

    #[test]
    fn test_sensor_injection() {
        let platform = SitlPlatform::new(VehicleId(1));

        // No sensors initially
        assert!(platform.take_sensors().is_none());

        // Inject sensor data
        let data = SensorData {
            timestamp_us: 1000,
            vehicle_id: VehicleId(1),
            imu: Some(ImuData {
                accel_mss: [0.0, 0.0, -9.81],
                gyro_rads: [0.0, 0.0, 0.0],
                temperature_c: 25.0,
            }),
            gps: None,
            compass: None,
            barometer: None,
            attitude_quat: None,
        };
        platform.inject_sensors(&data);

        // Take consumes the data
        let taken = platform.take_sensors();
        assert!(taken.is_some());
        assert_eq!(taken.unwrap().timestamp_us, 1000);

        // Second take returns None
        assert!(platform.take_sensors().is_none());
    }

    #[test]
    fn test_actuator_collection() {
        let platform = SitlPlatform::new(VehicleId(1));
        let idx0 = platform.create_pwm(0, 50).unwrap();
        let idx1 = platform.create_pwm(1, 50).unwrap();

        // Set duty cycles (0.5 = neutral = 0.0 motor output)
        platform.set_pwm_duty(idx0, 0.75); // -> 0.5 motor
        platform.set_pwm_duty(idx1, 0.25); // -> -0.5 motor

        let commands = platform.collect_actuator_commands();
        assert_eq!(commands.vehicle_id, VehicleId(1));
        assert_eq!(commands.motors.len(), 2);
        assert!((commands.motors[0] - 0.5).abs() < f32::EPSILON);
        assert!((commands.motors[1] - (-0.5)).abs() < f32::EPSILON);
    }

    #[test]
    fn test_battery_adc() {
        let platform = SitlPlatform::new(VehicleId(1));
        assert_eq!(platform.read_battery_adc(), 0);

        platform.set_battery_adc_value(3000);
        assert_eq!(platform.read_battery_adc(), 3000);
    }

    #[test]
    fn test_shared_time_source() {
        let time = SitlTimeSource::new();
        let platform = SitlPlatform::with_time_source(VehicleId(1), time.clone());

        time.advance_us(5000);
        assert_eq!(platform.time_source().now_us(), 5000);
    }

    #[test]
    fn test_gps_sensor_injection() {
        let platform = SitlPlatform::new(VehicleId(1));

        let data = SensorData {
            timestamp_us: 2000,
            vehicle_id: VehicleId(1),
            imu: None,
            gps: Some(GpsData {
                lat_deg: 35.6762,
                lon_deg: 139.6503,
                alt_m: 40.0,
                speed_ms: 1.5,
                course_deg: 90.0,
                fix_type: GpsFixType::Fix3D,
                satellites: 12,
                hdop: 0.8,
            }),
            compass: None,
            barometer: None,
            attitude_quat: None,
        };
        platform.inject_sensors(&data);

        let taken = platform.take_sensors().unwrap();
        let gps = taken.gps.unwrap();
        assert!((gps.lat_deg - 35.6762).abs() < 1e-4);
    }
}
