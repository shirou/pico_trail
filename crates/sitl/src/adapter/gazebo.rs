use std::net::SocketAddr;
use std::time::Duration;

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use tokio::net::UdpSocket;
use tokio::time::timeout;

use super::capabilities::{SensorCapabilities, SimulatorCapabilities};
use super::SimulatorAdapter;
use crate::error::SimulatorError;
use crate::types::{
    ActuatorCommands, CompassData, GpsData, GpsFixType, ImuData, SensorData, VehicleId,
};

/// Configuration for the Gazebo adapter.
#[derive(Debug, Clone)]
pub struct GazeboConfig {
    /// UDP port for receiving sensor data (default: 9002).
    pub sensor_port: u16,
    /// UDP port for sending actuator commands (default: 9003).
    pub actuator_port: u16,
    /// Address of the Gazebo server.
    pub server_addr: SocketAddr,
    /// Use ardupilot_gazebo JSON protocol.
    pub ardupilot_compat: bool,
    /// Timeout in milliseconds for UDP receive operations.
    pub timeout_ms: u32,
}

impl Default for GazeboConfig {
    fn default() -> Self {
        Self {
            sensor_port: 9002,
            actuator_port: 9003,
            server_addr: "127.0.0.1:9003".parse().unwrap(),
            ardupilot_compat: true,
            timeout_ms: 1000,
        }
    }
}

/// JSON format for sensor data from ardupilot_gazebo.
#[derive(Debug, Deserialize)]
struct GazeboSensorJson {
    timestamp: u64,
    imu: Option<GazeboImuJson>,
    gps: Option<GazeboGpsJson>,
    compass: Option<GazeboCompassJson>,
}

#[derive(Debug, Deserialize)]
struct GazeboImuJson {
    linear_acceleration: [f32; 3],
    angular_velocity: [f32; 3],
}

#[derive(Debug, Deserialize)]
struct GazeboGpsJson {
    lat: f64,
    lon: f64,
    alt: f32,
    speed: f32,
    course: f32,
}

#[derive(Debug, Deserialize)]
struct GazeboCompassJson {
    mag: [f32; 3],
}

/// JSON format for actuator commands sent to Gazebo.
#[derive(Debug, Serialize)]
struct GazeboActuatorJson {
    timestamp: u64,
    motors: Vec<u16>,
}

/// Adapter that communicates with Gazebo Harmonic via UDP/JSON,
/// compatible with the ardupilot_gazebo plugin.
pub struct GazeboAdapter {
    config: GazeboConfig,
    name: String,
    vehicle_id: VehicleId,
    sensor_socket: Option<UdpSocket>,
    actuator_socket: Option<UdpSocket>,
    sim_time_us: u64,
    connected: bool,
    recv_buf: Vec<u8>,
}

impl GazeboAdapter {
    /// Create a new GazeboAdapter with the given configuration.
    pub fn new(name: impl Into<String>, vehicle_id: VehicleId, config: GazeboConfig) -> Self {
        Self {
            config,
            name: name.into(),
            vehicle_id,
            sensor_socket: None,
            actuator_socket: None,
            sim_time_us: 0,
            connected: false,
            recv_buf: vec![0u8; 4096],
        }
    }

    /// Create a new GazeboAdapter with default configuration.
    pub fn with_defaults(name: impl Into<String>, vehicle_id: VehicleId) -> Self {
        Self::new(name, vehicle_id, GazeboConfig::default())
    }

    /// Parse ardupilot_gazebo JSON sensor data into `SensorData`.
    fn parse_sensor_json(&self, json: &str) -> Result<SensorData, SimulatorError> {
        let parsed: GazeboSensorJson = serde_json::from_str(json)
            .map_err(|e| SimulatorError::ProtocolError(format!("JSON parse error: {e}")))?;

        let imu = parsed.imu.map(|imu| ImuData {
            accel_mss: imu.linear_acceleration,
            gyro_rads: imu.angular_velocity,
            temperature_c: 25.0, // Gazebo doesn't provide IMU temperature
        });

        let gps = parsed.gps.map(|gps| GpsData {
            lat_deg: gps.lat,
            lon_deg: gps.lon,
            alt_m: gps.alt,
            speed_ms: gps.speed,
            course_deg: gps.course,
            fix_type: GpsFixType::Fix3D,
            satellites: 12,
            hdop: 0.8,
        });

        let compass = parsed.compass.map(|c| CompassData { mag_gauss: c.mag });

        Ok(SensorData {
            timestamp_us: parsed.timestamp,
            vehicle_id: self.vehicle_id,
            imu,
            gps,
            compass,
            barometer: None,
        })
    }

    /// Serialize actuator commands to ardupilot_gazebo JSON format.
    fn serialize_actuator_json(&self, commands: &ActuatorCommands) -> String {
        let motors_pwm: Vec<u16> = commands
            .motors
            .iter()
            .map(|&v| normalized_to_pwm(v))
            .collect();

        let json = GazeboActuatorJson {
            timestamp: commands.timestamp_us,
            motors: motors_pwm,
        };

        serde_json::to_string(&json).unwrap_or_default()
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
        let sensor_addr: SocketAddr = ([0, 0, 0, 0], self.config.sensor_port).into();
        let sensor_socket = UdpSocket::bind(sensor_addr).await.map_err(|e| {
            SimulatorError::ConnectionFailed(format!(
                "Failed to bind sensor socket on port {}: {e}",
                self.config.sensor_port
            ))
        })?;

        let actuator_addr: SocketAddr = ([0, 0, 0, 0], 0).into();
        let actuator_socket = UdpSocket::bind(actuator_addr).await.map_err(|e| {
            SimulatorError::ConnectionFailed(format!("Failed to bind actuator socket: {e}"))
        })?;

        actuator_socket
            .connect(self.config.server_addr)
            .await
            .map_err(|e| {
                SimulatorError::ConnectionFailed(format!(
                    "Failed to connect actuator socket to {}: {e}",
                    self.config.server_addr
                ))
            })?;

        self.sensor_socket = Some(sensor_socket);
        self.actuator_socket = Some(actuator_socket);
        self.sim_time_us = 0;
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), SimulatorError> {
        self.sensor_socket = None;
        self.actuator_socket = None;
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
            .sensor_socket
            .as_ref()
            .ok_or_else(|| SimulatorError::ConnectionFailed("Sensor socket not bound".into()))?;

        let recv_timeout = Duration::from_millis(self.config.timeout_ms as u64);

        match timeout(recv_timeout, socket.recv(&mut self.recv_buf)).await {
            Ok(Ok(len)) => {
                let json_str = std::str::from_utf8(&self.recv_buf[..len]).map_err(|e| {
                    SimulatorError::ProtocolError(format!("Invalid UTF-8 in sensor data: {e}"))
                })?;
                let sensor_data = self.parse_sensor_json(json_str)?;
                self.sim_time_us = sensor_data.timestamp_us;
                Ok(Some(sensor_data))
            }
            Ok(Err(e)) => Err(SimulatorError::Io(e)),
            Err(_) => Ok(None), // Timeout: no data available
        }
    }

    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError> {
        if !self.connected {
            return Err(SimulatorError::ConnectionFailed(
                "Not connected".to_string(),
            ));
        }

        let socket = self
            .actuator_socket
            .as_ref()
            .ok_or_else(|| SimulatorError::ConnectionFailed("Actuator socket not bound".into()))?;

        let json = self.serialize_actuator_json(commands);
        socket.send(json.as_bytes()).await?;
        Ok(())
    }

    async fn step(&mut self) -> Result<(), SimulatorError> {
        if !self.connected {
            return Err(SimulatorError::ConnectionFailed(
                "Not connected".to_string(),
            ));
        }
        // Gazebo advances time via its own physics engine.
        // In lockstep mode, we signal readiness by sending an empty step packet.
        // For now, sim_time_us is updated from received sensor data timestamps.
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

    // --- JSON parsing tests ---

    #[test]
    fn test_parse_sensor_json_full() {
        let adapter = create_adapter();
        let json = r#"{
            "timestamp": 1000000,
            "imu": {
                "linear_acceleration": [0.1, 0.2, -9.81],
                "angular_velocity": [0.01, 0.02, 0.03]
            },
            "gps": {
                "lat": 35.6762,
                "lon": 139.6503,
                "alt": 40.0,
                "speed": 1.5,
                "course": 90.0
            }
        }"#;

        let result = adapter.parse_sensor_json(json).unwrap();
        assert_eq!(result.timestamp_us, 1_000_000);
        assert_eq!(result.vehicle_id, VehicleId(1));

        let imu = result.imu.unwrap();
        assert!((imu.accel_mss[0] - 0.1).abs() < f32::EPSILON);
        assert!((imu.accel_mss[2] - (-9.81)).abs() < 0.001);
        assert!((imu.gyro_rads[0] - 0.01).abs() < f32::EPSILON);

        let gps = result.gps.unwrap();
        assert!((gps.lat_deg - 35.6762).abs() < 1e-6);
        assert!((gps.lon_deg - 139.6503).abs() < 1e-6);
        assert!((gps.alt_m - 40.0).abs() < f32::EPSILON);
        assert!((gps.speed_ms - 1.5).abs() < f32::EPSILON);
        assert!((gps.course_deg - 90.0).abs() < f32::EPSILON);
    }

    #[test]
    fn test_parse_sensor_json_imu_only() {
        let adapter = create_adapter();
        let json = r#"{
            "timestamp": 500000,
            "imu": {
                "linear_acceleration": [0.0, 0.0, -9.81],
                "angular_velocity": [0.0, 0.0, 0.0]
            }
        }"#;

        let result = adapter.parse_sensor_json(json).unwrap();
        assert!(result.imu.is_some());
        assert!(result.gps.is_none());
        assert!(result.compass.is_none());
        assert!(result.barometer.is_none());
    }

    #[test]
    fn test_parse_sensor_json_gps_only() {
        let adapter = create_adapter();
        let json = r#"{
            "timestamp": 500000,
            "gps": {
                "lat": 35.0,
                "lon": 139.0,
                "alt": 10.0,
                "speed": 0.0,
                "course": 0.0
            }
        }"#;

        let result = adapter.parse_sensor_json(json).unwrap();
        assert!(result.imu.is_none());
        assert!(result.gps.is_some());
    }

    #[test]
    fn test_parse_sensor_json_with_compass() {
        let adapter = create_adapter();
        let json = r#"{
            "timestamp": 500000,
            "compass": {
                "mag": [0.25, -0.1, 0.45]
            }
        }"#;

        let result = adapter.parse_sensor_json(json).unwrap();
        assert!(result.imu.is_none());
        assert!(result.gps.is_none());
        let compass = result.compass.unwrap();
        assert!((compass.mag_gauss[0] - 0.25).abs() < f32::EPSILON);
        assert!((compass.mag_gauss[1] - (-0.1)).abs() < f32::EPSILON);
        assert!((compass.mag_gauss[2] - 0.45).abs() < f32::EPSILON);
    }

    #[test]
    fn test_parse_sensor_json_full_with_compass() {
        let adapter = create_adapter();
        let json = r#"{
            "timestamp": 1000000,
            "imu": {
                "linear_acceleration": [0.0, 0.0, -9.81],
                "angular_velocity": [0.0, 0.0, 0.0]
            },
            "gps": {
                "lat": 35.0,
                "lon": 139.0,
                "alt": 10.0,
                "speed": 0.0,
                "course": 0.0
            },
            "compass": {
                "mag": [0.2, -0.05, 0.4]
            }
        }"#;

        let result = adapter.parse_sensor_json(json).unwrap();
        assert!(result.imu.is_some());
        assert!(result.gps.is_some());
        assert!(result.compass.is_some());
        assert!(result.barometer.is_none());
    }

    #[test]
    fn test_parse_sensor_json_timestamp_only() {
        let adapter = create_adapter();
        let json = r#"{"timestamp": 0}"#;

        let result = adapter.parse_sensor_json(json).unwrap();
        assert_eq!(result.timestamp_us, 0);
        assert!(result.imu.is_none());
        assert!(result.gps.is_none());
    }

    #[test]
    fn test_parse_sensor_json_malformed() {
        let adapter = create_adapter();
        let result = adapter.parse_sensor_json("not json at all");
        assert!(result.is_err());
        match result.unwrap_err() {
            SimulatorError::ProtocolError(msg) => {
                assert!(msg.contains("JSON parse error"));
            }
            other => panic!("Expected ProtocolError, got: {other:?}"),
        }
    }

    #[test]
    fn test_parse_sensor_json_missing_timestamp() {
        let adapter = create_adapter();
        let json = r#"{"imu": {"linear_acceleration": [0,0,0], "angular_velocity": [0,0,0]}}"#;
        let result = adapter.parse_sensor_json(json);
        assert!(result.is_err());
    }

    // --- JSON serialization tests ---

    #[test]
    fn test_serialize_actuator_json() {
        let adapter = create_adapter();
        let commands = ActuatorCommands {
            timestamp_us: 1_000_000,
            vehicle_id: VehicleId(1),
            motors: vec![0.0, 0.0, 0.0, 0.0],
            servos: vec![],
        };

        let json_str = adapter.serialize_actuator_json(&commands);
        let parsed: serde_json::Value = serde_json::from_str(&json_str).unwrap();
        assert_eq!(parsed["timestamp"], 1_000_000);
        let motors = parsed["motors"].as_array().unwrap();
        assert_eq!(motors.len(), 4);
        for m in motors {
            assert_eq!(m, 1500); // neutral PWM
        }
    }

    #[test]
    fn test_serialize_actuator_json_full_range() {
        let adapter = create_adapter();
        let commands = ActuatorCommands {
            timestamp_us: 0,
            vehicle_id: VehicleId(1),
            motors: vec![-1.0, 1.0],
            servos: vec![],
        };

        let json_str = adapter.serialize_actuator_json(&commands);
        let parsed: serde_json::Value = serde_json::from_str(&json_str).unwrap();
        let motors = parsed["motors"].as_array().unwrap();
        assert_eq!(motors[0], 1000);
        assert_eq!(motors[1], 2000);
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
        assert!(caps.sensors.compass);
        assert!(!caps.sensors.barometer);
        assert!(caps.multi_vehicle);
        assert!(caps.terrain);
        assert!(caps.wind);
        assert_eq!(caps.max_rate_hz, 1000);
    }

    #[test]
    fn test_default_config() {
        let config = GazeboConfig::default();
        assert_eq!(config.sensor_port, 9002);
        assert_eq!(config.actuator_port, 9003);
        assert!(config.ardupilot_compat);
        assert_eq!(config.timeout_ms, 1000);
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
        assert!(adapter.sensor_socket.is_none());
        assert!(adapter.actuator_socket.is_none());
    }
}
