use async_trait::async_trait;
use pico_trail_sitl::{
    ActuatorCommands, SensorData, SimulatorAdapter, SimulatorCapabilities, SimulatorError,
    SitlBridge, TimeMode, VehicleConfig, VehicleId, VehicleType,
};

/// Mock adapter for bridge testing.
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
fn register_and_list_adapters() {
    let mut bridge = SitlBridge::new();
    assert!(bridge.list_adapters().is_empty());

    bridge
        .register_adapter(Box::new(MockAdapter::new("sim1")))
        .unwrap();
    let adapters = bridge.list_adapters();
    assert_eq!(adapters.len(), 1);
    assert!(adapters.contains(&"sim1"));
}

#[test]
fn duplicate_adapter_name_errors() {
    let mut bridge = SitlBridge::new();
    bridge
        .register_adapter(Box::new(MockAdapter::new("sim1")))
        .unwrap();

    let result = bridge.register_adapter(Box::new(MockAdapter::new("sim1")));
    assert!(result.is_err());
}

#[test]
fn unregister_adapter() {
    let mut bridge = SitlBridge::new();
    bridge
        .register_adapter(Box::new(MockAdapter::new("sim1")))
        .unwrap();
    bridge.unregister_adapter("sim1").unwrap();
    assert!(bridge.list_adapters().is_empty());
}

#[test]
fn unregister_missing_adapter_errors() {
    let mut bridge = SitlBridge::new();
    let result = bridge.unregister_adapter("nonexistent");
    assert!(result.is_err());
}

#[test]
fn spawn_and_list_vehicles() {
    let mut bridge = SitlBridge::new();
    let config = VehicleConfig::new(VehicleId(1), VehicleType::Rover);
    let id = bridge.spawn_vehicle(config).unwrap();
    assert_eq!(id, VehicleId(1));

    let vehicles = bridge.list_vehicles();
    assert_eq!(vehicles.len(), 1);
    assert!(vehicles.contains(&VehicleId(1)));
}

#[test]
fn duplicate_vehicle_id_errors() {
    let mut bridge = SitlBridge::new();
    let config1 = VehicleConfig::new(VehicleId(1), VehicleType::Rover);
    let config2 = VehicleConfig::new(VehicleId(1), VehicleType::Rover);

    bridge.spawn_vehicle(config1).unwrap();
    let result = bridge.spawn_vehicle(config2);
    assert!(result.is_err());
}

#[test]
fn despawn_vehicle() {
    let mut bridge = SitlBridge::new();
    let config = VehicleConfig::new(VehicleId(1), VehicleType::Rover);
    bridge.spawn_vehicle(config).unwrap();
    bridge.despawn_vehicle(VehicleId(1)).unwrap();
    assert!(bridge.list_vehicles().is_empty());
}

#[test]
fn despawn_missing_vehicle_errors() {
    let mut bridge = SitlBridge::new();
    let result = bridge.despawn_vehicle(VehicleId(99));
    assert!(result.is_err());
}

#[test]
fn assign_vehicle_to_adapter() {
    let mut bridge = SitlBridge::new();
    bridge
        .register_adapter(Box::new(MockAdapter::new("sim1")))
        .unwrap();
    let config = VehicleConfig::new(VehicleId(1), VehicleType::Rover);
    bridge.spawn_vehicle(config).unwrap();

    bridge
        .assign_vehicle_to_adapter(VehicleId(1), "sim1")
        .unwrap();
}

#[test]
fn assign_vehicle_to_missing_adapter_errors() {
    let mut bridge = SitlBridge::new();
    let config = VehicleConfig::new(VehicleId(1), VehicleType::Rover);
    bridge.spawn_vehicle(config).unwrap();

    let result = bridge.assign_vehicle_to_adapter(VehicleId(1), "nonexistent");
    assert!(result.is_err());
}

#[test]
fn assign_missing_vehicle_errors() {
    let mut bridge = SitlBridge::new();
    bridge
        .register_adapter(Box::new(MockAdapter::new("sim1")))
        .unwrap();

    let result = bridge.assign_vehicle_to_adapter(VehicleId(99), "sim1");
    assert!(result.is_err());
}

#[test]
fn get_adapter() {
    let mut bridge = SitlBridge::new();
    bridge
        .register_adapter(Box::new(MockAdapter::new("sim1")))
        .unwrap();

    let adapter = bridge.get_adapter("sim1");
    assert!(adapter.is_some());
    assert_eq!(adapter.unwrap().name(), "sim1");

    assert!(bridge.get_adapter("nonexistent").is_none());
}

#[test]
fn time_mode_default_is_free_running() {
    let bridge = SitlBridge::new();
    assert_eq!(bridge.sim_time_us(), 0);
}

#[tokio::test]
async fn lockstep_step_advances_time() {
    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Lockstep { step_size_us: 1000 });

    bridge.step().await.unwrap();
    assert_eq!(bridge.sim_time_us(), 1000);

    bridge.step().await.unwrap();
    assert_eq!(bridge.sim_time_us(), 2000);
}

#[test]
fn error_types_constructible() {
    let _ = SimulatorError::ConnectionFailed("test".to_string());
    let _ = SimulatorError::ProtocolError("test".to_string());
    let _ = SimulatorError::Timeout("test");
    let _ = SimulatorError::AdapterNotFound("test".to_string());
    let _ = SimulatorError::AdapterAlreadyRegistered("test".to_string());
    let _ = SimulatorError::VehicleNotFound(VehicleId(1));
    let _ = SimulatorError::VehicleAlreadyExists(VehicleId(1));
    let _ = SimulatorError::VehicleNotAssigned(VehicleId(1));
}

#[test]
fn error_display_messages() {
    let err = SimulatorError::ConnectionFailed("timeout".to_string());
    assert_eq!(format!("{err}"), "Connection failed: timeout");

    let err = SimulatorError::VehicleNotFound(VehicleId(5));
    assert!(format!("{err}").contains("5"));
}

#[test]
fn vehicle_config_default_port() {
    let config = VehicleConfig::new(VehicleId(3), VehicleType::Rover);
    assert_eq!(config.mavlink_port, 14553); // 14550 + 3
    assert_eq!(config.vehicle_type, VehicleType::Rover);
    assert!(config.initial_position.is_none());
}
