//! AHRS async task for Embassy executor
//!
//! Runs at 100Hz to update attitude estimate from IMU data.
//! Publishes results to shared state for other subsystems.

use super::{CalibrationData, Dcm, DcmConfig, SharedAttitudeState};
use crate::devices::traits::ImuReading;
use nalgebra::Vector3;

/// IMU data from sensors
///
/// This is the interface between IMU driver and AHRS subsystem.
/// The IMU driver should populate this struct and pass it to AHRS.
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope reading in rad/s (body frame)
    pub gyro: Vector3<f32>,

    /// Accelerometer reading in m/s² (body frame, includes gravity)
    pub accel: Vector3<f32>,

    /// Magnetometer reading in µT (body frame), optional
    pub mag: Option<Vector3<f32>>,

    /// Timestamp in milliseconds since startup
    pub timestamp_ms: u64,
}

impl From<ImuReading> for ImuData {
    fn from(reading: ImuReading) -> Self {
        Self {
            gyro: reading.gyro,
            accel: reading.accel,
            mag: Some(reading.mag),
            timestamp_ms: reading.timestamp_us / 1000,
        }
    }
}

/// AHRS task configuration
pub struct AhrsTaskConfig {
    /// DCM algorithm configuration (PI gains)
    pub dcm_config: DcmConfig,

    /// IMU update rate in Hz (default: 100)
    pub imu_rate_hz: u32,

    /// Magnetometer update rate in Hz (default: 10)
    pub mag_rate_hz: u32,

    /// Convergence time in seconds (default: 5.0)
    pub convergence_time_s: f32,
}

impl Default for AhrsTaskConfig {
    fn default() -> Self {
        Self {
            dcm_config: DcmConfig::default(),
            imu_rate_hz: 100,
            mag_rate_hz: 10,
            convergence_time_s: 5.0,
        }
    }
}

/// AHRS task state (internal to task)
struct AhrsTaskState {
    dcm: Dcm,
    calibration: CalibrationData,
    start_time_ms: u64,
    mag_update_counter: u32,
    converged: bool,
}

impl AhrsTaskState {
    fn new(config: DcmConfig, calibration: CalibrationData) -> Self {
        let mut dcm = Dcm::new(config);
        dcm.set_gyro_bias(calibration.gyro_bias);

        Self {
            dcm,
            calibration,
            start_time_ms: 0,
            mag_update_counter: 0,
            converged: false,
        }
    }

    /// Process IMU update
    fn process_imu(&mut self, imu: ImuData, dt: f32) -> (f32, f32, f32) {
        // Apply calibration to sensor readings
        let gyro_calibrated = imu.gyro - self.calibration.gyro_bias;
        let accel_calibrated = self.calibration.apply_accel_calibration(imu.accel);

        // Update DCM with gyro and accel
        self.dcm.update(gyro_calibrated, accel_calibrated, dt);

        // Get current attitude
        self.dcm.get_euler_angles()
    }

    /// Process magnetometer update (called less frequently)
    fn process_mag(&mut self, mag: Vector3<f32>, dt: f32) {
        let mag_calibrated = self.calibration.apply_mag_calibration(mag);
        self.dcm.update_with_mag(mag_calibrated, dt);
    }

    /// Check and update convergence status
    fn update_convergence(&mut self, current_time_ms: u64, convergence_time_s: f32) -> bool {
        if self.start_time_ms == 0 {
            self.start_time_ms = current_time_ms;
        }

        if !self.converged {
            let elapsed_s = (current_time_ms - self.start_time_ms) as f32 / 1000.0;
            if elapsed_s >= convergence_time_s {
                self.converged = true;
            }
        }

        self.converged
    }
}

/// AHRS task entry point (for Embassy executor)
///
/// This is a placeholder implementation showing the structure.
/// In actual hardware, this would be called by the Embassy executor.
///
/// # Arguments
///
/// * `state` - Shared attitude state to publish results
/// * `config` - AHRS task configuration
/// * `calibration` - IMU calibration data
/// * `imu_reader` - Async function to read IMU data
///
/// # Example (conceptual)
///
/// ```ignore
/// #[embassy_executor::task]
/// async fn ahrs_task(
///     state: &'static SharedAttitudeState,
///     imu: impl ImuDriver,
/// ) {
///     let config = AhrsTaskConfig::default();
///     let calibration = CalibrationData::default();
///
///     run_ahrs_task(state, config, calibration, || async {
///         imu.read().await
///     }).await
/// }
/// ```
pub async fn run_ahrs_task<F, Fut>(
    shared_state: &SharedAttitudeState,
    config: AhrsTaskConfig,
    calibration: CalibrationData,
    mut imu_reader: F,
) where
    F: FnMut() -> Fut,
    Fut: core::future::Future<Output = ImuData>,
{
    let mut task_state = AhrsTaskState::new(config.dcm_config, calibration);

    let dt = 1.0 / config.imu_rate_hz as f32;
    let mag_decimation = config.imu_rate_hz / config.mag_rate_hz;

    loop {
        // Read IMU data
        let imu = imu_reader().await;

        // Process IMU update
        let (roll, pitch, yaw) = task_state.process_imu(imu, dt);

        // Process magnetometer (decimated to lower rate)
        if task_state.mag_update_counter >= mag_decimation {
            task_state.mag_update_counter = 0;
            if let Some(mag) = imu.mag {
                let mag_dt = mag_decimation as f32 * dt;
                task_state.process_mag(mag, mag_dt);
            }
        }
        task_state.mag_update_counter += 1;

        // Update convergence status
        let converged = task_state.update_convergence(imu.timestamp_ms, config.convergence_time_s);

        // Publish to shared state
        shared_state.update_attitude(roll, pitch, yaw, imu.timestamp_ms);
        shared_state.set_converged(converged);

        // Note: In real implementation, timing would be controlled by
        // Embassy timer or task scheduler to maintain 100Hz rate
    }
}

/// IMU task configuration for 400Hz sampling
pub struct ImuTaskConfig {
    /// Sampling rate in Hz (default: 400)
    pub sample_rate_hz: u32,

    /// Latency logging interval in samples (default: 400 = 1 second)
    pub log_interval: u32,

    /// Maximum acceptable jitter in microseconds (default: 500)
    pub max_jitter_us: u32,
}

impl Default for ImuTaskConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 400,
            log_interval: 400,
            max_jitter_us: 500,
        }
    }
}

/// IMU task entry point for 400Hz sensor reading
///
/// This task reads from an `ImuSensor` implementation at 400Hz and
/// provides data for the EKF AHRS system (to be implemented in T-p8w8f).
///
/// # Arguments
///
/// * `config` - IMU task configuration
/// * `imu` - An implementation of the ImuSensor trait (e.g., Mpu9250Driver)
/// * `callback` - Async callback to process each ImuData reading
///
/// # Example
///
/// ```ignore
/// use pico_trail::devices::imu::Mpu9250Driver;
/// use pico_trail::subsystems::ahrs::{run_imu_task, ImuTaskConfig};
///
/// #[embassy_executor::task]
/// async fn imu_task(imu: Mpu9250Driver) {
///     let config = ImuTaskConfig::default();
///     run_imu_task(config, imu, |data| async move {
///         // Pass to EKF or log data
///     }).await;
/// }
/// ```
#[cfg(feature = "embassy")]
pub async fn run_imu_task<I, F, Fut>(config: ImuTaskConfig, mut imu: I, mut callback: F)
where
    I: crate::devices::traits::ImuSensor,
    F: FnMut(ImuData) -> Fut,
    Fut: core::future::Future<Output = ()>,
{
    use embassy_time::{Duration, Instant, Ticker};

    let period_us = 1_000_000 / config.sample_rate_hz;
    let mut ticker = Ticker::every(Duration::from_micros(period_us as u64));

    let mut sample_count: u32 = 0;
    let mut total_latency_us: u64 = 0;
    let mut max_latency_us: u64 = 0;

    loop {
        ticker.next().await;
        let start = Instant::now();

        // Read from IMU sensor
        match imu.read_all().await {
            Ok(reading) => {
                let imu_data = ImuData::from(reading);
                callback(imu_data).await;

                // Track latency
                let latency = start.elapsed().as_micros();
                total_latency_us += latency;
                if latency > max_latency_us {
                    max_latency_us = latency;
                }

                sample_count += 1;

                // Log statistics periodically
                if sample_count >= config.log_interval {
                    let avg_latency = total_latency_us / sample_count as u64;
                    crate::log_debug!(
                        "IMU: {} samples, avg={}us, max={}us",
                        sample_count,
                        avg_latency,
                        max_latency_us
                    );

                    // Reset counters
                    sample_count = 0;
                    total_latency_us = 0;
                    max_latency_us = 0;
                }
            }
            Err(e) => {
                crate::log_warn!("IMU read error: {:?}", e);
                // Continue running, sensor may recover
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ahrs_task_config_default() {
        let config = AhrsTaskConfig::default();
        assert_eq!(config.imu_rate_hz, 100);
        assert_eq!(config.mag_rate_hz, 10);
        assert_eq!(config.convergence_time_s, 5.0);
    }

    #[test]
    fn test_imu_data_creation() {
        let imu = ImuData {
            gyro: Vector3::new(0.01, -0.02, 0.005),
            accel: Vector3::new(0.0, 0.0, 9.81),
            mag: Some(Vector3::new(20.0, 5.0, -40.0)),
            timestamp_ms: 1000,
        };

        assert_eq!(imu.gyro.x, 0.01);
        assert_eq!(imu.accel.z, 9.81);
        assert!(imu.mag.is_some());
    }

    #[test]
    fn test_task_state_initialization() {
        let config = DcmConfig::default();
        let calibration = CalibrationData {
            gyro_bias: Vector3::new(0.01, -0.005, 0.002),
            ..Default::default()
        };

        let task_state = AhrsTaskState::new(config, calibration);

        // Should have initialized DCM with gyro bias
        assert_eq!(task_state.dcm.gyro_bias(), calibration.gyro_bias);
        assert!(!task_state.converged);
    }

    #[test]
    fn test_convergence_detection() {
        let config = DcmConfig::default();
        let calibration = CalibrationData::default();
        let mut task_state = AhrsTaskState::new(config, calibration);

        // Initially not converged (start from 100ms to avoid 0 edge case)
        assert!(!task_state.update_convergence(100, 5.0));
        assert!(!task_state.update_convergence(1100, 5.0)); // 1s elapsed
        assert!(!task_state.update_convergence(4100, 5.0)); // 4s elapsed

        // Should converge at 5s
        assert!(task_state.update_convergence(5100, 5.0)); // 5s elapsed
        assert!(task_state.update_convergence(6100, 5.0)); // Stays converged
    }

    #[test]
    fn test_mag_decimation() {
        let config = AhrsTaskConfig {
            imu_rate_hz: 100,
            mag_rate_hz: 10,
            ..Default::default()
        };

        let decimation = config.imu_rate_hz / config.mag_rate_hz;
        assert_eq!(decimation, 10); // Update mag every 10 IMU samples
    }
}
