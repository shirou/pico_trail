//! Lightweight differential drive simulator adapter.
//!
//! Built-in physics simulation with no external dependencies, suitable for
//! CI testing and rapid iteration. Implements differential drive kinematics
//! with configurable sensor noise and deterministic mode.

use async_trait::async_trait;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::adapter::capabilities::{SensorCapabilities, SimulatorCapabilities};
use crate::adapter::SimulatorAdapter;
use crate::error::SimulatorError;
use crate::types::{
    ActuatorCommands, CompassData, GpsData, GpsFixType, ImuData, SensorData, VehicleId,
};

/// Configuration for the lightweight simulator.
#[derive(Debug, Clone)]
pub struct LightweightConfig {
    /// Distance between wheels in meters.
    pub wheel_base: f32,
    /// Maximum vehicle speed in m/s.
    pub max_speed: f32,
    /// Maximum turn rate in rad/s.
    pub max_turn_rate: f32,
    /// GPS position noise standard deviation in meters.
    pub gps_noise_m: f32,
    /// GPS update rate in Hz.
    pub gps_rate_hz: u32,
    /// Accelerometer noise standard deviation in m/s².
    pub accel_noise_mss: f32,
    /// Gyroscope noise standard deviation in rad/s.
    pub gyro_noise_rads: f32,
    /// Compass noise standard deviation in radians.
    pub compass_noise_rad: f32,
    /// RNG seed for deterministic mode. None = random.
    pub seed: Option<u64>,
    /// Simulation step size in microseconds.
    pub step_size_us: u64,
    /// Reference latitude for GPS synthesis (degrees).
    pub origin_lat_deg: f64,
    /// Reference longitude for GPS synthesis (degrees).
    pub origin_lon_deg: f64,
}

impl Default for LightweightConfig {
    fn default() -> Self {
        Self {
            wheel_base: 0.15,
            max_speed: 1.0,
            max_turn_rate: 2.0,
            gps_noise_m: 0.5,
            gps_rate_hz: 5,
            accel_noise_mss: 0.1,
            gyro_noise_rads: 0.01,
            compass_noise_rad: 0.05,
            seed: None,
            step_size_us: 10_000, // 100 Hz
            origin_lat_deg: 35.6762,
            origin_lon_deg: 139.6503,
        }
    }
}

/// Internal vehicle state for kinematics integration.
#[derive(Debug, Clone)]
struct VehicleState {
    /// X position in meters (east).
    x: f32,
    /// Y position in meters (north).
    y: f32,
    /// Heading in radians (0 = east, pi/2 = north).
    heading: f32,
    /// Forward velocity in m/s.
    velocity: f32,
    /// Angular velocity in rad/s.
    angular_velocity: f32,
    /// Left motor command (-1.0 to 1.0).
    motor_left: f32,
    /// Right motor command (-1.0 to 1.0).
    motor_right: f32,
    /// Previous velocity for acceleration calculation.
    prev_velocity: f32,
    /// Previous angular velocity for angular acceleration.
    prev_angular_velocity: f32,
}

impl VehicleState {
    fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            velocity: 0.0,
            angular_velocity: 0.0,
            motor_left: 0.0,
            motor_right: 0.0,
            prev_velocity: 0.0,
            prev_angular_velocity: 0.0,
        }
    }
}

/// Lightweight simulator adapter with built-in differential drive kinematics.
///
/// Provides a self-contained simulation without external dependencies,
/// suitable for CI and unit testing.
pub struct LightweightAdapter {
    config: LightweightConfig,
    name: String,
    vehicle_id: VehicleId,
    state: VehicleState,
    rng: StdRng,
    sim_time_us: u64,
    connected: bool,
    step_count: u64,
}

impl LightweightAdapter {
    /// Create a new lightweight adapter with the given configuration.
    pub fn new(name: &str, vehicle_id: VehicleId, config: LightweightConfig) -> Self {
        let rng = match config.seed {
            Some(seed) => StdRng::seed_from_u64(seed),
            None => StdRng::from_entropy(),
        };
        Self {
            config,
            name: name.to_string(),
            vehicle_id,
            state: VehicleState::new(),
            rng,
            sim_time_us: 0,
            connected: false,
            step_count: 0,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults(vehicle_id: VehicleId) -> Self {
        Self::new("lightweight", vehicle_id, LightweightConfig::default())
    }

    /// Integrate differential drive kinematics for one time step.
    fn integrate(&mut self, dt: f32) {
        let v_left = self.state.motor_left * self.config.max_speed;
        let v_right = self.state.motor_right * self.config.max_speed;

        self.state.prev_velocity = self.state.velocity;
        self.state.prev_angular_velocity = self.state.angular_velocity;

        // Differential drive model
        self.state.velocity = (v_left + v_right) / 2.0;
        self.state.angular_velocity = (v_right - v_left) / self.config.wheel_base;

        // Clamp turn rate
        self.state.angular_velocity = self
            .state
            .angular_velocity
            .clamp(-self.config.max_turn_rate, self.config.max_turn_rate);

        // Update heading
        self.state.heading += self.state.angular_velocity * dt;
        // Normalize heading to [-pi, pi]
        self.state.heading = normalize_angle(self.state.heading);

        // Update position
        self.state.x += self.state.velocity * self.state.heading.cos() * dt;
        self.state.y += self.state.velocity * self.state.heading.sin() * dt;
    }

    /// Synthesize sensor data from current vehicle state.
    fn synthesize_sensors(&mut self) -> SensorData {
        let imu = self.synthesize_imu();
        let gps = self.synthesize_gps();
        let compass = self.synthesize_compass();

        // Convert math-convention heading (0=east, CCW) to NED yaw (0=north, CW)
        let ned_yaw = core::f32::consts::FRAC_PI_2 - self.state.heading;
        // Quaternion [w, x, y, z] for pure yaw rotation (level vehicle)
        let half_yaw = ned_yaw / 2.0;
        let attitude_quat = Some([half_yaw.cos(), 0.0, 0.0, half_yaw.sin()]);

        SensorData {
            timestamp_us: self.sim_time_us,
            vehicle_id: self.vehicle_id,
            imu: Some(imu),
            gps,
            compass: Some(compass),
            barometer: None,
            attitude_quat,
        }
    }

    /// Synthesize IMU data from motion state.
    fn synthesize_imu(&mut self) -> ImuData {
        let dt = self.config.step_size_us as f32 / 1_000_000.0;
        let dt = if dt > 0.0 { dt } else { 0.01 };

        // Linear acceleration from velocity change
        let accel_forward = (self.state.velocity - self.state.prev_velocity) / dt;
        // Centripetal acceleration
        let accel_lateral = self.state.velocity * self.state.angular_velocity;

        // Body-frame accelerations (x=forward, y=left, z=up)
        let ax = accel_forward + self.gaussian_noise(self.config.accel_noise_mss);
        let ay = -accel_lateral + self.gaussian_noise(self.config.accel_noise_mss);
        // Gravity on Z axis
        let az = -9.81 + self.gaussian_noise(self.config.accel_noise_mss);

        // Gyroscope (angular velocity around z-axis for 2D)
        let gz = self.state.angular_velocity + self.gaussian_noise(self.config.gyro_noise_rads);

        ImuData {
            accel_mss: [ax, ay, az],
            gyro_rads: [
                self.gaussian_noise(self.config.gyro_noise_rads),
                self.gaussian_noise(self.config.gyro_noise_rads),
                gz,
            ],
            temperature_c: 25.0,
        }
    }

    /// Synthesize GPS data from position (rate-limited).
    fn synthesize_gps(&mut self) -> Option<GpsData> {
        if self.config.gps_rate_hz == 0 {
            return None;
        }

        let steps_per_gps = 1_000_000 / (self.config.step_size_us * self.config.gps_rate_hz as u64);
        let steps_per_gps = steps_per_gps.max(1);

        if !self.step_count.is_multiple_of(steps_per_gps) {
            return None;
        }

        // Convert local x,y to lat/lon using flat-earth approximation
        let meters_per_deg_lat = 111_320.0_f64;
        let meters_per_deg_lon = 111_320.0_f64 * (self.config.origin_lat_deg.to_radians().cos());

        let noise_x = self.gaussian_noise(self.config.gps_noise_m);
        let noise_y = self.gaussian_noise(self.config.gps_noise_m);

        let lat =
            self.config.origin_lat_deg + ((self.state.y + noise_y) as f64) / meters_per_deg_lat;
        let lon =
            self.config.origin_lon_deg + ((self.state.x + noise_x) as f64) / meters_per_deg_lon;

        // Speed and course
        let speed = self.state.velocity.abs();
        let course_deg = self.state.heading.to_degrees();
        let course_deg = if course_deg < 0.0 {
            course_deg + 360.0
        } else {
            course_deg
        };

        Some(GpsData {
            lat_deg: lat,
            lon_deg: lon,
            alt_m: 0.0,
            speed_ms: speed,
            course_deg,
            fix_type: GpsFixType::Fix3D,
            satellites: 12,
            hdop: 0.8,
        })
    }

    /// Synthesize compass data from heading.
    fn synthesize_compass(&mut self) -> CompassData {
        let heading_with_noise =
            self.state.heading + self.gaussian_noise(self.config.compass_noise_rad);

        // Magnetic field vector in body frame (simplified: horizontal only)
        // Assumes magnetic north = geographic north for simplicity
        let mag_x = heading_with_noise.cos();
        let mag_y = heading_with_noise.sin();

        CompassData {
            mag_gauss: [mag_x, mag_y, 0.0],
        }
    }

    /// Generate Gaussian noise using Box-Muller transform.
    fn gaussian_noise(&mut self, stddev: f32) -> f32 {
        if stddev == 0.0 {
            return 0.0;
        }
        let u1: f32 = self.rng.gen::<f32>().max(f32::EPSILON);
        let u2: f32 = self.rng.gen();
        let z = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos();
        z * stddev
    }

    /// Get the current vehicle state position.
    pub fn position(&self) -> (f32, f32) {
        (self.state.x, self.state.y)
    }

    /// Get the current heading in radians.
    pub fn heading(&self) -> f32 {
        self.state.heading
    }

    /// Get the current velocity in m/s.
    pub fn velocity(&self) -> f32 {
        self.state.velocity
    }
}

impl std::fmt::Debug for LightweightAdapter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LightweightAdapter")
            .field("name", &self.name)
            .field("vehicle_id", &self.vehicle_id)
            .field("connected", &self.connected)
            .field("sim_time_us", &self.sim_time_us)
            .finish()
    }
}

#[async_trait]
impl SimulatorAdapter for LightweightAdapter {
    fn adapter_type(&self) -> &'static str {
        "lightweight"
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn connect(&mut self) -> Result<(), SimulatorError> {
        // Reset state on connect
        self.state = VehicleState::new();
        self.sim_time_us = 0;
        self.step_count = 0;
        self.rng = match self.config.seed {
            Some(seed) => StdRng::seed_from_u64(seed),
            None => StdRng::from_entropy(),
        };
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), SimulatorError> {
        self.connected = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError> {
        if !self.connected {
            return Err(SimulatorError::ConnectionFailed(
                "Not connected".to_string(),
            ));
        }
        Ok(Some(self.synthesize_sensors()))
    }

    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError> {
        if !self.connected {
            return Err(SimulatorError::ConnectionFailed(
                "Not connected".to_string(),
            ));
        }
        // Map motor commands to left/right differential drive
        if commands.motors.len() >= 2 {
            self.state.motor_left = commands.motors[0].clamp(-1.0, 1.0);
            self.state.motor_right = commands.motors[1].clamp(-1.0, 1.0);
        } else if commands.motors.len() == 1 {
            // Single motor: drive straight
            self.state.motor_left = commands.motors[0].clamp(-1.0, 1.0);
            self.state.motor_right = commands.motors[0].clamp(-1.0, 1.0);
        }
        Ok(())
    }

    async fn step(&mut self) -> Result<(), SimulatorError> {
        if !self.connected {
            return Err(SimulatorError::ConnectionFailed(
                "Not connected".to_string(),
            ));
        }
        let dt = self.config.step_size_us as f32 / 1_000_000.0;
        self.integrate(dt);
        self.sim_time_us += self.config.step_size_us;
        self.step_count += 1;
        Ok(())
    }

    fn sim_time_us(&self) -> u64 {
        self.sim_time_us
    }

    fn supports_lockstep(&self) -> bool {
        true
    }

    fn capabilities(&self) -> SimulatorCapabilities {
        SimulatorCapabilities {
            sensors: SensorCapabilities {
                imu: true,
                gps: true,
                compass: true,
                barometer: false,
            },
            max_rate_hz: 1_000_000 / self.config.step_size_us as u32,
            multi_vehicle: false,
            terrain: false,
            wind: false,
        }
    }
}

/// Normalize angle to [-pi, pi].
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % (2.0 * std::f32::consts::PI);
    if a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    } else if a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_adapter(seed: u64) -> LightweightAdapter {
        let config = LightweightConfig {
            seed: Some(seed),
            gps_noise_m: 0.0,
            accel_noise_mss: 0.0,
            gyro_noise_rads: 0.0,
            compass_noise_rad: 0.0,
            step_size_us: 10_000, // 100 Hz
            ..Default::default()
        };
        LightweightAdapter::new("test", VehicleId(1), config)
    }

    fn create_noisy_adapter(seed: u64) -> LightweightAdapter {
        let config = LightweightConfig {
            seed: Some(seed),
            step_size_us: 10_000,
            ..Default::default()
        };
        LightweightAdapter::new("test-noisy", VehicleId(1), config)
    }

    #[tokio::test]
    async fn test_straight_line_motion() {
        let mut adapter = create_test_adapter(42);
        adapter.connect().await.unwrap();

        // Both motors forward at 50%
        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![0.5, 0.5],
            servos: vec![],
        };
        adapter.send_actuators(&commands).await.unwrap();

        // Run for 1 second (100 steps at 100 Hz)
        for _ in 0..100 {
            adapter.step().await.unwrap();
        }

        let (x, y) = adapter.position();
        // Expected: v = 0.5 * 1.0 = 0.5 m/s, heading = 0 (east)
        // After 1s: x ≈ 0.5m, y ≈ 0.0m
        assert!((x - 0.5).abs() < 0.01, "Expected x ≈ 0.5, got {}", x);
        assert!(y.abs() < 0.01, "Expected y ≈ 0.0, got {}", y);
    }

    #[tokio::test]
    async fn test_rotation_in_place() {
        let mut adapter = create_test_adapter(42);
        adapter.connect().await.unwrap();

        // Left motor backward, right motor forward (rotate in place)
        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![-0.5, 0.5],
            servos: vec![],
        };
        adapter.send_actuators(&commands).await.unwrap();

        // Run for 100 steps
        for _ in 0..100 {
            adapter.step().await.unwrap();
        }

        let (x, y) = adapter.position();
        // For rotation in place: v = 0 (motors cancel out)
        // Position should stay near origin
        assert!(x.abs() < 0.01, "Expected x ≈ 0.0, got {}", x);
        assert!(y.abs() < 0.01, "Expected y ≈ 0.0, got {}", y);

        // Heading should have changed significantly
        // omega = (v_right - v_left) / wheel_base = (0.5 - (-0.5)) * 1.0 / 0.15 = 6.67 rad/s
        // But clamped to max_turn_rate = 2.0 rad/s
        // After 1s: heading ≈ 2.0 rad
        let heading = adapter.heading();
        assert!(
            (heading - 2.0).abs() < 0.1,
            "Expected heading ≈ 2.0 rad, got {}",
            heading
        );
    }

    #[tokio::test]
    async fn test_arc_turn() {
        let mut adapter = create_test_adapter(42);
        adapter.connect().await.unwrap();

        // Asymmetric motors: left slower than right -> turn left
        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![0.3, 0.5],
            servos: vec![],
        };
        adapter.send_actuators(&commands).await.unwrap();

        for _ in 0..100 {
            adapter.step().await.unwrap();
        }

        let (x, y) = adapter.position();
        let heading = adapter.heading();

        // Vehicle should have moved forward and turned
        assert!(x > 0.0, "Vehicle should have moved in x");
        // With heading turning positive (counterclockwise), y should be positive
        assert!(heading > 0.0, "Heading should be positive (turning left)");
        // Position should not be on x-axis
        assert!(y.abs() > 0.01, "Vehicle should have curved off x-axis");
    }

    #[tokio::test]
    async fn test_deterministic_mode() {
        // Run the same simulation twice with the same seed
        async fn run_simulation(seed: u64) -> (f32, f32, f32) {
            let mut adapter = create_noisy_adapter(seed);
            adapter.connect().await.unwrap();

            let commands = ActuatorCommands {
                timestamp_us: 0,
                vehicle_id: VehicleId(1),
                motors: vec![0.4, 0.6],
                servos: vec![],
            };
            adapter.send_actuators(&commands).await.unwrap();

            for _ in 0..50 {
                adapter.step().await.unwrap();
            }

            let sensors = adapter.receive_sensors().await.unwrap().unwrap();
            let imu = sensors.imu.unwrap();
            (adapter.position().0, adapter.position().1, imu.accel_mss[0])
        }

        let (x1, y1, a1) = run_simulation(12345).await;
        let (x2, y2, a2) = run_simulation(12345).await;

        assert_eq!(x1, x2, "Deterministic: x positions must match");
        assert_eq!(y1, y2, "Deterministic: y positions must match");
        assert_eq!(a1, a2, "Deterministic: IMU readings must match");
    }

    #[tokio::test]
    async fn test_noise_varies_output() {
        let mut adapter1 = create_noisy_adapter(100);
        let mut adapter2 = create_noisy_adapter(200);

        adapter1.connect().await.unwrap();
        adapter2.connect().await.unwrap();

        // Same motor commands
        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![0.5, 0.5],
            servos: vec![],
        };
        adapter1.send_actuators(&commands).await.unwrap();
        adapter2.send_actuators(&commands).await.unwrap();

        adapter1.step().await.unwrap();
        adapter2.step().await.unwrap();

        let s1 = adapter1.receive_sensors().await.unwrap().unwrap();
        let s2 = adapter2.receive_sensors().await.unwrap().unwrap();

        // With different seeds, noisy sensor readings should differ
        let imu1 = s1.imu.unwrap();
        let imu2 = s2.imu.unwrap();
        assert_ne!(
            imu1.accel_mss[0], imu2.accel_mss[0],
            "Different seeds should produce different noise"
        );
    }

    #[tokio::test]
    async fn test_gps_rate_limiting() {
        let config = LightweightConfig {
            seed: Some(42),
            gps_noise_m: 0.0,
            gps_rate_hz: 5,       // 5 Hz GPS
            step_size_us: 10_000, // 100 Hz sim
            accel_noise_mss: 0.0,
            gyro_noise_rads: 0.0,
            compass_noise_rad: 0.0,
            ..Default::default()
        };
        let mut adapter = LightweightAdapter::new("test", VehicleId(1), config);
        adapter.connect().await.unwrap();

        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![0.5, 0.5],
            servos: vec![],
        };
        adapter.send_actuators(&commands).await.unwrap();

        // Count GPS readings over 40 steps (0.4s at 100Hz)
        let mut gps_count = 0;
        for _ in 0..40 {
            adapter.step().await.unwrap();
            let sensors = adapter.receive_sensors().await.unwrap().unwrap();
            if sensors.gps.is_some() {
                gps_count += 1;
            }
        }

        // At 5 Hz GPS and 100 Hz sim: expect ~2 GPS readings in 0.4s
        assert!(
            (1..=4).contains(&gps_count),
            "Expected ~2 GPS readings, got {}",
            gps_count
        );
    }

    #[tokio::test]
    async fn test_connect_resets_state() {
        let mut adapter = create_test_adapter(42);
        adapter.connect().await.unwrap();

        // Move the vehicle
        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![1.0, 1.0],
            servos: vec![],
        };
        adapter.send_actuators(&commands).await.unwrap();
        for _ in 0..100 {
            adapter.step().await.unwrap();
        }
        assert!(adapter.position().0 > 0.0);

        // Reconnect should reset
        adapter.connect().await.unwrap();
        let (x, y) = adapter.position();
        assert_eq!(x, 0.0);
        assert_eq!(y, 0.0);
        assert_eq!(adapter.sim_time_us(), 0);
    }

    #[tokio::test]
    async fn test_adapter_type_and_capabilities() {
        let adapter = create_test_adapter(42);
        assert_eq!(adapter.adapter_type(), "lightweight");
        assert!(adapter.supports_lockstep());

        let caps = adapter.capabilities();
        assert!(caps.sensors.imu);
        assert!(caps.sensors.gps);
        assert!(caps.sensors.compass);
        assert!(!caps.sensors.barometer);
        assert!(!caps.multi_vehicle);
    }

    #[tokio::test]
    async fn test_not_connected_errors() {
        let mut adapter = create_test_adapter(42);
        // Not connected yet
        assert!(!adapter.is_connected());
        assert!(adapter.step().await.is_err());
        assert!(adapter.receive_sensors().await.is_err());
    }

    #[tokio::test]
    async fn test_single_motor_drives_straight() {
        let mut adapter = create_test_adapter(42);
        adapter.connect().await.unwrap();

        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![0.5],
            servos: vec![],
        };
        adapter.send_actuators(&commands).await.unwrap();

        for _ in 0..100 {
            adapter.step().await.unwrap();
        }

        // Single motor should drive both wheels equally (straight line)
        let (x, y) = adapter.position();
        assert!(x > 0.0, "Should move forward");
        assert!(y.abs() < 0.01, "Should go straight");
    }
}
