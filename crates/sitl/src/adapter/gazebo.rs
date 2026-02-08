use std::net::SocketAddr;
use std::time::Duration;

use async_trait::async_trait;
use serde::Deserialize;
use tokio::net::UdpSocket;
use tokio::time::timeout;

use super::capabilities::{SensorCapabilities, SimulatorCapabilities};
use super::SimulatorAdapter;
use crate::error::SimulatorError;
use crate::types::{ActuatorCommands, GpsData, GpsFixType, ImuData, SensorData, VehicleId};

// --- ArduPilot SITL protocol constants ---

/// Magic number for 16-channel servo packets (ardupilot_gazebo expects 18458 = 0x481A).
const SERVO_MAGIC_16: u16 = 0x481A;

/// Default frame rate sent in servo packets.
const DEFAULT_FRAME_RATE: u16 = 400;

/// Earth radius in meters (WGS84 approximation).
const EARTH_RADIUS_M: f64 = 6_371_000.0;

// --- JSON FDM response structures (from ardupilot_gazebo plugin) ---

#[derive(Debug, Deserialize)]
struct FdmJson {
    timestamp: f64,
    imu: FdmImu,
    position: [f64; 3],
    velocity: [f64; 3],
    quaternion: [f64; 4],
}

#[derive(Debug, Deserialize)]
struct FdmImu {
    gyro: [f64; 3],
    accel_body: [f64; 3],
}

/// Configuration for the Gazebo adapter.
#[derive(Debug, Clone)]
pub struct GazeboConfig {
    /// Address and port of Gazebo's ArduPilotPlugin (`fdm_port_in`).
    /// We send servo packets here; Gazebo responds with FDM data to our source port.
    pub gazebo_addr: SocketAddr,
    /// Timeout in milliseconds for UDP receive operations.
    pub timeout_ms: u32,
    /// GPS origin latitude (degrees). NED position offsets are relative to this.
    pub origin_lat_deg: f64,
    /// GPS origin longitude (degrees).
    pub origin_lon_deg: f64,
    /// GPS origin altitude (meters above sea level).
    pub origin_alt_m: f64,
}

impl Default for GazeboConfig {
    fn default() -> Self {
        Self {
            gazebo_addr: "127.0.0.1:9002".parse().unwrap(),
            timeout_ms: 1000,
            // Default origin: Tokyo Station
            origin_lat_deg: 35.681236,
            origin_lon_deg: 139.767125,
            origin_alt_m: 40.0,
        }
    }
}

/// Adapter that communicates with Gazebo Harmonic via the `ardupilot_gazebo` plugin.
///
/// **Servo packet (us → Gazebo):** 40-byte binary with magic 0x481A,
/// frame rate, frame count, and 16 PWM channels.
///
/// **FDM response (Gazebo → us):** JSON string with timestamp, IMU (gyro, accel),
/// position (NED meters), velocity (NED m/s), and quaternion.
///
/// Position in NED meters is converted to GPS lat/lon using the configured origin.
pub struct GazeboAdapter {
    config: GazeboConfig,
    name: String,
    vehicle_id: VehicleId,
    socket: Option<UdpSocket>,
    sim_time_us: u64,
    connected: bool,
    frame_count: u32,
    recv_buf: Vec<u8>,
}

impl GazeboAdapter {
    /// Create a new GazeboAdapter with the given configuration.
    pub fn new(name: impl Into<String>, vehicle_id: VehicleId, config: GazeboConfig) -> Self {
        Self {
            config,
            name: name.into(),
            vehicle_id,
            socket: None,
            sim_time_us: 0,
            connected: false,
            frame_count: 0,
            recv_buf: vec![0u8; 2048],
        }
    }

    /// Create a new GazeboAdapter with default configuration.
    pub fn with_defaults(name: impl Into<String>, vehicle_id: VehicleId) -> Self {
        Self::new(name, vehicle_id, GazeboConfig::default())
    }

    /// Build a 40-byte servo_packet from actuator commands.
    fn build_servo_packet(&mut self, commands: &ActuatorCommands) -> [u8; 40] {
        let mut buf = [0u8; 40];

        // magic: u16 LE
        buf[0..2].copy_from_slice(&SERVO_MAGIC_16.to_le_bytes());
        // frame_rate: u16 LE
        buf[2..4].copy_from_slice(&DEFAULT_FRAME_RATE.to_le_bytes());
        // frame_count: u32 LE
        buf[4..8].copy_from_slice(&self.frame_count.to_le_bytes());
        self.frame_count = self.frame_count.wrapping_add(1);

        // pwm[16]: u16 LE, fill from motor commands, rest = 0 (disarmed)
        for (i, &motor_val) in commands.motors.iter().enumerate().take(16) {
            let pwm = normalized_to_pwm(motor_val);
            let offset = 8 + i * 2;
            buf[offset..offset + 2].copy_from_slice(&pwm.to_le_bytes());
        }

        buf
    }

    /// Parse a JSON FDM response from the ardupilot_gazebo plugin into SensorData.
    fn parse_fdm_json(&self, data: &[u8]) -> Result<SensorData, SimulatorError> {
        let fdm: FdmJson = serde_json::from_slice(data)
            .map_err(|e| SimulatorError::ProtocolError(format!("FDM JSON parse error: {e}")))?;

        let timestamp_us = (fdm.timestamp * 1_000_000.0) as u64;

        let imu = ImuData {
            gyro_rads: [
                fdm.imu.gyro[0] as f32,
                fdm.imu.gyro[1] as f32,
                fdm.imu.gyro[2] as f32,
            ],
            accel_mss: [
                fdm.imu.accel_body[0] as f32,
                fdm.imu.accel_body[1] as f32,
                fdm.imu.accel_body[2] as f32,
            ],
            temperature_c: 25.0,
        };

        // The ardupilot_gazebo plugin sends the quaternion as earth-to-body
        // rotation in NED frame. We need body-to-earth, so take the conjugate
        // (negate x, y, z components).
        let attitude_quat = Some([
            fdm.quaternion[0] as f32,
            -(fdm.quaternion[1] as f32),
            -(fdm.quaternion[2] as f32),
            -(fdm.quaternion[3] as f32),
        ]);

        // Velocity NED [north, east, down] m/s
        let vel_n = fdm.velocity[0];
        let vel_e = fdm.velocity[1];
        let speed_ms = (vel_n * vel_n + vel_e * vel_e).sqrt();
        let course_deg = vel_e.atan2(vel_n).to_degrees();
        let course_deg = if course_deg < 0.0 {
            course_deg + 360.0
        } else {
            course_deg
        };

        // Position NED [north, east, down] meters from Gazebo origin
        let pos_n = fdm.position[0];
        let pos_e = fdm.position[1];
        let pos_d = fdm.position[2];

        // Convert NED offset to lat/lon
        let lat_deg = self.config.origin_lat_deg + (pos_n / EARTH_RADIUS_M).to_degrees();
        let lon_deg = self.config.origin_lon_deg
            + (pos_e / (EARTH_RADIUS_M * self.config.origin_lat_deg.to_radians().cos()))
                .to_degrees();
        let alt_m = self.config.origin_alt_m - pos_d; // NED: down is positive

        let gps = GpsData {
            lat_deg,
            lon_deg,
            alt_m: alt_m as f32,
            speed_ms: speed_ms as f32,
            course_deg: course_deg as f32,
            fix_type: GpsFixType::Fix3D,
            satellites: 12,
            hdop: 0.8,
        };

        Ok(SensorData {
            timestamp_us,
            vehicle_id: self.vehicle_id,
            imu: Some(imu),
            gps: Some(gps),
            compass: None,
            barometer: None,
            attitude_quat,
        })
    }
}

impl std::fmt::Debug for GazeboAdapter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GazeboAdapter")
            .field("name", &self.name)
            .field("vehicle_id", &self.vehicle_id)
            .field("connected", &self.connected)
            .field("sim_time_us", &self.sim_time_us)
            .finish()
    }
}

#[async_trait]
impl SimulatorAdapter for GazeboAdapter {
    fn adapter_type(&self) -> &'static str {
        "gazebo"
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn connect(&mut self) -> Result<(), SimulatorError> {
        let local_addr: SocketAddr = ([0, 0, 0, 0], 0).into();
        let socket = UdpSocket::bind(local_addr).await.map_err(|e| {
            SimulatorError::ConnectionFailed(format!("Failed to bind UDP socket: {e}"))
        })?;

        // Connect the UDP socket to the target so recv() works
        socket.connect(self.config.gazebo_addr).await.map_err(|e| {
            SimulatorError::ConnectionFailed(format!("Failed to connect UDP socket: {e}"))
        })?;

        println!(
            "  [{}] UDP connected {} -> {}",
            self.name,
            socket.local_addr().unwrap(),
            self.config.gazebo_addr
        );

        self.socket = Some(socket);
        self.sim_time_us = 0;
        self.frame_count = 0;
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), SimulatorError> {
        self.socket = None;
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

        let socket = self
            .socket
            .as_ref()
            .ok_or_else(|| SimulatorError::ConnectionFailed("Socket not bound".into()))?;

        let recv_timeout = Duration::from_millis(self.config.timeout_ms as u64);

        // DEBUG: track receive attempts
        static DEBUG_COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let debug_n = DEBUG_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);

        match timeout(recv_timeout, socket.recv(&mut self.recv_buf)).await {
            Ok(Ok(len)) if len > 0 => {
                if debug_n < 10 {
                    let preview = String::from_utf8_lossy(&self.recv_buf[..len.min(100)]);
                    eprintln!("  [{}] RX {} bytes: {}", self.name, len, preview);
                }
                let sensor_data = self.parse_fdm_json(&self.recv_buf[..len])?;
                self.sim_time_us = sensor_data.timestamp_us;
                Ok(Some(sensor_data))
            }
            Ok(Ok(_)) => {
                if debug_n < 10 {
                    eprintln!("  [{}] RX 0 bytes (empty)", self.name);
                }
                Ok(None)
            }
            Ok(Err(e)) => {
                if debug_n < 10 {
                    eprintln!("  [{}] RX error: {e}", self.name);
                }
                Err(SimulatorError::Io(e))
            }
            Err(_) => {
                if debug_n < 10 {
                    eprintln!(
                        "  [{}] RX timeout ({}ms)",
                        self.name, self.config.timeout_ms
                    );
                }
                Ok(None)
            }
        }
    }

    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError> {
        if !self.connected {
            return Err(SimulatorError::ConnectionFailed(
                "Not connected".to_string(),
            ));
        }

        let packet = self.build_servo_packet(commands);
        let socket = self
            .socket
            .as_ref()
            .ok_or_else(|| SimulatorError::ConnectionFailed("Socket not bound".into()))?;

        // DEBUG: log first few sends
        if self.frame_count <= 3 {
            let hex: Vec<String> = packet[..8].iter().map(|b| format!("{b:02x}")).collect();
            eprintln!(
                "  [{}] TX #{} to {} ({} bytes) magic=0x{:04X} hdr: {}",
                self.name,
                self.frame_count - 1,
                self.config.gazebo_addr,
                packet.len(),
                SERVO_MAGIC_16,
                hex.join(" ")
            );
        }

        socket.send(&packet).await?;
        Ok(())
    }

    async fn step(&mut self) -> Result<(), SimulatorError> {
        if !self.connected {
            return Err(SimulatorError::ConnectionFailed(
                "Not connected".to_string(),
            ));
        }
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
                compass: false,
                barometer: false,
            },
            max_rate_hz: 1000,
            multi_vehicle: true,
            terrain: true,
            wind: true,
        }
    }
}

/// Convert a normalized motor value [-1.0, 1.0] to PWM [1000, 2000].
pub fn normalized_to_pwm(normalized: f32) -> u16 {
    let clamped = normalized.clamp(-1.0, 1.0);
    ((clamped + 1.0) * 500.0 + 1000.0) as u16
}

/// Convert a PWM value [1000, 2000] to normalized [-1.0, 1.0].
pub fn pwm_to_normalized(pwm: u16) -> f32 {
    let clamped = pwm.clamp(1000, 2000);
    (clamped as f32 - 1000.0) / 500.0 - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_adapter() -> GazeboAdapter {
        GazeboAdapter::new("test-gazebo", VehicleId(1), GazeboConfig::default())
    }

    /// Build a sample FDM JSON string for testing.
    fn sample_fdm_json(
        timestamp: f64,
        gyro: [f64; 3],
        accel: [f64; 3],
        position: [f64; 3],
        velocity: [f64; 3],
    ) -> String {
        serde_json::json!({
            "timestamp": timestamp,
            "imu": {
                "gyro": gyro,
                "accel_body": accel
            },
            "position": position,
            "quaternion": [1.0, 0.0, 0.0, 0.0],
            "velocity": velocity
        })
        .to_string()
    }

    // --- PWM conversion tests ---

    #[test]
    fn test_pwm_conversion_neutral() {
        assert_eq!(normalized_to_pwm(0.0), 1500);
        assert!((pwm_to_normalized(1500) - 0.0).abs() < f32::EPSILON);
    }

    #[test]
    fn test_pwm_conversion_full_forward() {
        assert_eq!(normalized_to_pwm(1.0), 2000);
        assert!((pwm_to_normalized(2000) - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn test_pwm_conversion_full_reverse() {
        assert_eq!(normalized_to_pwm(-1.0), 1000);
        assert!((pwm_to_normalized(1000) - (-1.0)).abs() < f32::EPSILON);
    }

    #[test]
    fn test_pwm_conversion_clamping() {
        assert_eq!(normalized_to_pwm(2.0), 2000);
        assert_eq!(normalized_to_pwm(-2.0), 1000);
        assert!((pwm_to_normalized(500) - (-1.0)).abs() < f32::EPSILON);
        assert!((pwm_to_normalized(3000) - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn test_pwm_roundtrip() {
        for pwm in (1000..=2000).step_by(100) {
            let normalized = pwm_to_normalized(pwm);
            let back = normalized_to_pwm(normalized);
            assert_eq!(back, pwm);
        }
    }

    // --- Servo packet tests ---

    #[test]
    fn test_build_servo_packet() {
        let mut adapter = create_adapter();
        let commands = ActuatorCommands {
            timestamp_us: 1_000_000,
            vehicle_id: VehicleId(1),
            motors: vec![0.0, 0.5, -1.0, 1.0],
            servos: vec![],
        };

        let packet = adapter.build_servo_packet(&commands);
        assert_eq!(packet.len(), 40);

        // Magic: 0x481A
        let magic = u16::from_le_bytes([packet[0], packet[1]]);
        assert_eq!(magic, SERVO_MAGIC_16);
        assert_eq!(magic, 0x481A);

        // Frame rate
        let rate = u16::from_le_bytes([packet[2], packet[3]]);
        assert_eq!(rate, DEFAULT_FRAME_RATE);

        // Frame count (first call = 0, incremented after)
        let count = u32::from_le_bytes([packet[4], packet[5], packet[6], packet[7]]);
        assert_eq!(count, 0);

        // PWM values
        let pwm0 = u16::from_le_bytes([packet[8], packet[9]]);
        assert_eq!(pwm0, 1500); // 0.0 → neutral

        let pwm1 = u16::from_le_bytes([packet[10], packet[11]]);
        assert_eq!(pwm1, 1750); // 0.5 → 1750

        let pwm2 = u16::from_le_bytes([packet[12], packet[13]]);
        assert_eq!(pwm2, 1000); // -1.0 → min

        let pwm3 = u16::from_le_bytes([packet[14], packet[15]]);
        assert_eq!(pwm3, 2000); // 1.0 → max

        // Remaining channels should be 0
        let pwm4 = u16::from_le_bytes([packet[16], packet[17]]);
        assert_eq!(pwm4, 0);
    }

    #[test]
    fn test_build_servo_packet_frame_count_increments() {
        let mut adapter = create_adapter();
        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![],
            servos: vec![],
        };

        let p1 = adapter.build_servo_packet(&commands);
        let c1 = u32::from_le_bytes([p1[4], p1[5], p1[6], p1[7]]);
        assert_eq!(c1, 0);

        let p2 = adapter.build_servo_packet(&commands);
        let c2 = u32::from_le_bytes([p2[4], p2[5], p2[6], p2[7]]);
        assert_eq!(c2, 1);
    }

    // --- JSON FDM parsing tests ---

    #[test]
    fn test_parse_fdm_json() {
        let adapter = create_adapter();

        let json = sample_fdm_json(
            1.5,                 // timestamp = 1.5s
            [0.01, 0.02, 0.03],  // gyro
            [0.0, 0.0, -9.81],   // accel
            [100.0, 50.0, -5.0], // position NED: 100m N, 50m E, 5m up
            [1.0, 0.0, 0.0],     // velocity NED: 1 m/s north
        );

        let result = adapter.parse_fdm_json(json.as_bytes()).unwrap();
        assert_eq!(result.timestamp_us, 1_500_000);
        assert_eq!(result.vehicle_id, VehicleId(1));

        let imu = result.imu.unwrap();
        assert!((imu.gyro_rads[0] - 0.01).abs() < 1e-5);
        assert!((imu.accel_mss[2] - (-9.81)).abs() < 0.01);

        let gps = result.gps.unwrap();
        // 100m north → lat should increase
        assert!(gps.lat_deg > adapter.config.origin_lat_deg);
        // 50m east → lon should increase
        assert!(gps.lon_deg > adapter.config.origin_lon_deg);
        // -5m down → altitude should increase by 5m
        assert!((gps.alt_m - 45.0).abs() < 0.1);
        // Speed: 1.0 m/s north
        assert!((gps.speed_ms - 1.0).abs() < 0.01);
        // Course: 0° (north)
        assert!(gps.course_deg < 1.0 || gps.course_deg > 359.0);

        // Quaternion: identity [1,0,0,0] conjugated → [1,0,0,0] (still identity)
        let quat = result.attitude_quat.unwrap();
        assert!((quat[0] - 1.0).abs() < 1e-5);
        assert!(quat[1].abs() < 1e-5);
        assert!(quat[2].abs() < 1e-5);
        assert!(quat[3].abs() < 1e-5);
    }

    #[test]
    fn test_parse_fdm_json_invalid() {
        let adapter = create_adapter();
        let result = adapter.parse_fdm_json(b"not json");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_fdm_json_extra_fields_ignored() {
        let adapter = create_adapter();
        // ardupilot_gazebo may include optional fields like rng_1, windvane
        let json = r#"{
            "timestamp": 0.5,
            "imu": {"gyro": [0,0,0], "accel_body": [0,0,-9.81]},
            "position": [0,0,0],
            "quaternion": [1,0,0,0],
            "velocity": [0,0,0],
            "rng_1": 2.5,
            "windvane": {"direction": 0.0, "speed": 1.5}
        }"#;
        let result = adapter.parse_fdm_json(json.as_bytes());
        assert!(result.is_ok());
    }

    // --- Adapter state tests ---

    #[test]
    fn test_adapter_type_and_name() {
        let adapter = create_adapter();
        assert_eq!(adapter.adapter_type(), "gazebo");
        assert_eq!(adapter.name(), "test-gazebo");
    }

    #[test]
    fn test_initial_state() {
        let adapter = create_adapter();
        assert!(!adapter.is_connected());
        assert_eq!(adapter.sim_time_us(), 0);
        assert!(adapter.supports_lockstep());
    }

    #[test]
    fn test_capabilities() {
        let adapter = create_adapter();
        let caps = adapter.capabilities();
        assert!(caps.sensors.imu);
        assert!(caps.sensors.gps);
        assert!(!caps.sensors.compass);
        assert!(!caps.sensors.barometer);
        assert!(caps.multi_vehicle);
        assert!(caps.terrain);
        assert!(caps.wind);
        assert_eq!(caps.max_rate_hz, 1000);
    }

    #[test]
    fn test_default_config() {
        let config = GazeboConfig::default();
        assert_eq!(config.gazebo_addr, "127.0.0.1:9002".parse().unwrap());
        assert_eq!(config.timeout_ms, 1000);
        assert!((config.origin_lat_deg - 35.681236).abs() < 1e-6);
    }

    #[tokio::test]
    async fn test_not_connected_errors() {
        let mut adapter = create_adapter();

        let result = adapter.receive_sensors().await;
        assert!(result.is_err());

        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![0.0],
            servos: vec![],
        };
        let result = adapter.send_actuators(&commands).await;
        assert!(result.is_err());

        let result = adapter.step().await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_disconnect_clears_state() {
        let mut adapter = create_adapter();
        adapter.connected = true;
        adapter.sim_time_us = 1_000_000;

        adapter.disconnect().await.unwrap();
        assert!(!adapter.is_connected());
        assert!(adapter.socket.is_none());
    }
}
