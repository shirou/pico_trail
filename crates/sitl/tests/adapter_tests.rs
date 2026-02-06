use async_trait::async_trait;
use pico_trail_sitl::{
    ActuatorCommands, SensorData, SimulatorAdapter, SimulatorCapabilities, SimulatorError,
    VehicleId,
};

/// Minimal mock adapter for testing trait object safety.
struct MockAdapter {
    name: String,
    connected: bool,
}

impl MockAdapter {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            connected: false,
        }
    }
}

#[async_trait]
impl SimulatorAdapter for MockAdapter {
    fn adapter_type(&self) -> &'static str {
        "mock"
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn connect(&mut self) -> Result<(), SimulatorError> {
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
        Ok(None)
    }

    async fn send_actuators(&mut self, _commands: &ActuatorCommands) -> Result<(), SimulatorError> {
        Ok(())
    }

    async fn step(&mut self) -> Result<(), SimulatorError> {
        Ok(())
    }

    fn sim_time_us(&self) -> u64 {
        0
    }

    fn supports_lockstep(&self) -> bool {
        false
    }

    fn capabilities(&self) -> SimulatorCapabilities {
        SimulatorCapabilities::default()
    }
}

#[test]
fn trait_is_object_safe() {
    // This test verifies that SimulatorAdapter can be used as a trait object.
    let adapter: Box<dyn SimulatorAdapter> = Box::new(MockAdapter::new("test"));
    assert_eq!(adapter.adapter_type(), "mock");
    assert_eq!(adapter.name(), "test");
    assert!(!adapter.is_connected());
}

#[test]
fn vehicle_id_derives() {
    let a = VehicleId(1);
    let b = VehicleId(1);
    let c = VehicleId(2);

    // PartialEq, Eq
    assert_eq!(a, b);
    assert_ne!(a, c);

    // Clone, Copy
    let d = a;
    assert_eq!(a, d);

    // Debug
    let debug_str = format!("{:?}", a);
    assert!(debug_str.contains("1"));

    // Display
    let display_str = format!("{}", a);
    assert!(display_str.contains("1"));

    // Hash
    use std::collections::HashSet;
    let mut set = HashSet::new();
    set.insert(a);
    set.insert(b);
    assert_eq!(set.len(), 1);
    set.insert(c);
    assert_eq!(set.len(), 2);
}

#[test]
fn sensor_data_construction_and_clone() {
    use pico_trail_sitl::types::{GpsData, GpsFixType, ImuData};

    let data = SensorData {
        timestamp_us: 1000,
        vehicle_id: VehicleId(1),
        imu: Some(ImuData {
            accel_mss: [0.0, 0.0, -9.81],
            gyro_rads: [0.0, 0.0, 0.0],
            temperature_c: 25.0,
        }),
        gps: Some(GpsData {
            lat_deg: 35.6762,
            lon_deg: 139.6503,
            alt_m: 40.0,
            speed_ms: 0.0,
            course_deg: 0.0,
            fix_type: GpsFixType::Fix3D,
            satellites: 12,
            hdop: 1.0,
        }),
        compass: None,
        barometer: None,
    };

    let cloned = data.clone();
    assert_eq!(cloned.timestamp_us, 1000);
    assert_eq!(cloned.vehicle_id, VehicleId(1));
    assert!(cloned.imu.is_some());
    assert!(cloned.gps.is_some());
    assert!(cloned.compass.is_none());
    assert!(cloned.barometer.is_none());
}

#[test]
fn simulator_capabilities_default() {
    let caps = SimulatorCapabilities::default();
    assert!(caps.sensors.imu);
    assert!(caps.sensors.gps);
    assert!(caps.sensors.compass);
    assert!(caps.sensors.barometer);
    assert_eq!(caps.max_rate_hz, 400);
    assert!(!caps.multi_vehicle);
    assert!(!caps.terrain);
    assert!(!caps.wind);
}
