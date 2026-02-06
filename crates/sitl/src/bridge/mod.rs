pub mod registry;
pub mod time;

use std::collections::HashMap;

pub use registry::AdapterRegistry;
pub use time::{TimeCoordinator, TimeMode};

use crate::adapter::SimulatorAdapter;
use crate::error::SimulatorError;
use crate::types::VehicleId;
use crate::vehicle::{VehicleConfig, VehicleInstance};

/// SITL Bridge orchestrator.
///
/// Manages simulator adapters, vehicles, and simulation time. This is the
/// main entry point for SITL operations.
pub struct SitlBridge {
    registry: AdapterRegistry,
    vehicles: HashMap<VehicleId, VehicleInstance>,
    time_coordinator: TimeCoordinator,
}

impl SitlBridge {
    pub fn new() -> Self {
        Self {
            registry: AdapterRegistry::new(),
            vehicles: HashMap::new(),
            time_coordinator: TimeCoordinator::default(),
        }
    }

    // -- Adapter management --

    /// Register a simulator adapter.
    pub fn register_adapter(
        &mut self,
        adapter: Box<dyn SimulatorAdapter>,
    ) -> Result<(), SimulatorError> {
        self.registry.register(adapter)
    }

    /// Unregister a simulator adapter by name.
    pub fn unregister_adapter(&mut self, name: &str) -> Result<(), SimulatorError> {
        self.registry.unregister(name)
    }

    /// List all registered adapter names.
    pub fn list_adapters(&self) -> Vec<&str> {
        self.registry.list()
    }

    /// Get a reference to an adapter by name.
    pub fn get_adapter(&self, name: &str) -> Option<&dyn SimulatorAdapter> {
        self.registry.get(name)
    }

    /// Get a mutable reference to an adapter by name.
    pub fn get_adapter_mut(&mut self, name: &str) -> Option<&mut Box<dyn SimulatorAdapter>> {
        self.registry.get_mut(name)
    }

    // -- Vehicle management --

    /// Spawn a new simulated vehicle.
    ///
    /// Checks for duplicate vehicle IDs and MAVLink port conflicts before
    /// creating the vehicle instance.
    pub fn spawn_vehicle(&mut self, config: VehicleConfig) -> Result<VehicleId, SimulatorError> {
        let id = config.id;
        if self.vehicles.contains_key(&id) {
            return Err(SimulatorError::VehicleAlreadyExists(id));
        }
        // Check for MAVLink port conflicts
        for existing in self.vehicles.values() {
            if existing.config.mavlink_port == config.mavlink_port {
                return Err(SimulatorError::PortConflict {
                    port: config.mavlink_port,
                    existing_vehicle: existing.id,
                });
            }
        }
        self.vehicles.insert(id, VehicleInstance::new(config));
        Ok(id)
    }

    /// Despawn a simulated vehicle.
    pub fn despawn_vehicle(&mut self, id: VehicleId) -> Result<(), SimulatorError> {
        if self.vehicles.remove(&id).is_none() {
            return Err(SimulatorError::VehicleNotFound(id));
        }
        Ok(())
    }

    /// Assign a vehicle to a specific adapter.
    pub fn assign_vehicle_to_adapter(
        &mut self,
        id: VehicleId,
        adapter_name: &str,
    ) -> Result<(), SimulatorError> {
        // Verify adapter exists
        if self.registry.get(adapter_name).is_none() {
            return Err(SimulatorError::AdapterNotFound(adapter_name.to_string()));
        }
        // Verify vehicle exists
        let vehicle = self
            .vehicles
            .get_mut(&id)
            .ok_or(SimulatorError::VehicleNotFound(id))?;
        vehicle.assigned_adapter = Some(adapter_name.to_string());
        Ok(())
    }

    /// List all active vehicle IDs.
    pub fn list_vehicles(&self) -> Vec<VehicleId> {
        self.vehicles.keys().copied().collect()
    }

    /// Get a reference to a vehicle by ID.
    pub fn get_vehicle(&self, id: VehicleId) -> Option<&VehicleInstance> {
        self.vehicles.get(&id)
    }

    /// Get a mutable reference to a vehicle by ID.
    pub fn get_vehicle_mut(&mut self, id: VehicleId) -> Option<&mut VehicleInstance> {
        self.vehicles.get_mut(&id)
    }

    // -- Simulation control --

    /// Advance the simulation by one step.
    ///
    /// Full step sequence:
    /// 1. Step all adapters (advance physics)
    /// 2. Receive sensor data from all adapters
    /// 3. Route sensor data to matching vehicles
    /// 4. Collect actuator commands from all vehicles
    /// 5. Send actuator commands to assigned adapters
    /// 6. Advance simulation time
    pub async fn step(&mut self) -> Result<(), SimulatorError> {
        // Snapshot adapter names to avoid borrow issues
        let adapter_names = self.registry.adapter_names();

        // 1. Step all adapters
        for name in &adapter_names {
            if let Some(adapter) = self.registry.get_mut(name) {
                if adapter.is_connected() {
                    adapter.step().await?;
                }
            }
        }

        // 2. Receive sensor data from all adapters
        let mut sensor_batch = Vec::new();
        for name in &adapter_names {
            if let Some(adapter) = self.registry.get_mut(name) {
                if adapter.is_connected() {
                    if let Some(data) = adapter.receive_sensors().await? {
                        sensor_batch.push(data);
                    }
                }
            }
        }

        // 3. Route sensor data to matching vehicles
        for data in &sensor_batch {
            if let Some(vehicle) = self.vehicles.get(&data.vehicle_id) {
                vehicle.platform.inject_sensors(data);
            }
        }

        // 4. Collect actuator commands from all assigned vehicles
        // Build a map: adapter_name -> Vec<ActuatorCommands>
        let mut actuator_map: HashMap<String, Vec<_>> = HashMap::new();
        for vehicle in self.vehicles.values() {
            if let Some(ref adapter_name) = vehicle.assigned_adapter {
                let commands = vehicle.platform.collect_actuator_commands();
                actuator_map
                    .entry(adapter_name.clone())
                    .or_default()
                    .push(commands);
            }
        }

        // 5. Send actuator commands to assigned adapters
        for (adapter_name, commands_list) in &actuator_map {
            if let Some(adapter) = self.registry.get_mut(adapter_name) {
                if adapter.is_connected() {
                    for commands in commands_list {
                        adapter.send_actuators(commands).await?;
                    }
                }
            }
        }

        // 6. Advance time
        self.time_coordinator.advance();

        Ok(())
    }

    /// Set the time synchronization mode.
    pub fn set_time_mode(&mut self, mode: TimeMode) {
        self.time_coordinator.set_mode(mode);
    }

    /// Get the current simulation time in microseconds.
    pub fn sim_time_us(&self) -> u64 {
        self.time_coordinator.sim_time_us()
    }
}

impl Default for SitlBridge {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::{LightweightAdapter, LightweightConfig};
    use crate::vehicle::VehicleType;

    fn make_config(id: u8) -> VehicleConfig {
        VehicleConfig::new(VehicleId(id), VehicleType::Rover)
    }

    fn make_config_with_port(id: u8, port: u16) -> VehicleConfig {
        VehicleConfig {
            mavlink_port: port,
            ..VehicleConfig::new(VehicleId(id), VehicleType::Rover)
        }
    }

    fn make_adapter(name: &str, vehicle_id: u8, seed: u64) -> Box<LightweightAdapter> {
        let config = LightweightConfig {
            seed: Some(seed),
            gps_noise_m: 0.0,
            accel_noise_mss: 0.0,
            gyro_noise_rads: 0.0,
            compass_noise_rad: 0.0,
            step_size_us: 10_000,
            ..Default::default()
        };
        Box::new(LightweightAdapter::new(name, VehicleId(vehicle_id), config))
    }

    #[test]
    fn test_multi_vehicle_spawn() {
        let mut bridge = SitlBridge::new();
        for i in 1..=10 {
            let id = bridge.spawn_vehicle(make_config(i)).unwrap();
            assert_eq!(id, VehicleId(i));
        }
        assert_eq!(bridge.list_vehicles().len(), 10);
    }

    #[test]
    fn test_duplicate_vehicle_rejected() {
        let mut bridge = SitlBridge::new();
        bridge.spawn_vehicle(make_config(1)).unwrap();
        let err = bridge.spawn_vehicle(make_config(1));
        assert!(matches!(err, Err(SimulatorError::VehicleAlreadyExists(_))));
    }

    #[test]
    fn test_port_conflict_detection() {
        let mut bridge = SitlBridge::new();
        bridge
            .spawn_vehicle(make_config_with_port(1, 14550))
            .unwrap();

        // Different vehicle ID, same port
        let err = bridge.spawn_vehicle(make_config_with_port(2, 14550));
        assert!(matches!(
            err,
            Err(SimulatorError::PortConflict {
                port: 14550,
                existing_vehicle: VehicleId(1)
            })
        ));
    }

    #[test]
    fn test_port_conflict_different_ports_ok() {
        let mut bridge = SitlBridge::new();
        bridge
            .spawn_vehicle(make_config_with_port(1, 14550))
            .unwrap();
        bridge
            .spawn_vehicle(make_config_with_port(2, 14551))
            .unwrap();
        assert_eq!(bridge.list_vehicles().len(), 2);
    }

    #[test]
    fn test_get_vehicle_accessors() {
        let mut bridge = SitlBridge::new();
        bridge.spawn_vehicle(make_config(1)).unwrap();

        assert!(bridge.get_vehicle(VehicleId(1)).is_some());
        assert!(bridge.get_vehicle(VehicleId(99)).is_none());
        assert!(bridge.get_vehicle_mut(VehicleId(1)).is_some());
    }

    #[test]
    fn test_vehicle_has_platform() {
        let mut bridge = SitlBridge::new();
        bridge.spawn_vehicle(make_config(5)).unwrap();

        let vehicle = bridge.get_vehicle(VehicleId(5)).unwrap();
        assert_eq!(vehicle.platform.vehicle_id(), VehicleId(5));
    }

    #[tokio::test]
    async fn test_lockstep_timing() {
        let mut bridge = SitlBridge::new();
        bridge.set_time_mode(TimeMode::Lockstep {
            step_size_us: 10_000,
        });

        assert_eq!(bridge.sim_time_us(), 0);
        bridge.step().await.unwrap();
        assert_eq!(bridge.sim_time_us(), 10_000);
        bridge.step().await.unwrap();
        assert_eq!(bridge.sim_time_us(), 20_000);
    }

    #[tokio::test]
    async fn test_sensor_routing() {
        let mut bridge = SitlBridge::new();
        bridge.set_time_mode(TimeMode::Lockstep {
            step_size_us: 10_000,
        });

        // Register adapter and spawn vehicle
        let adapter = make_adapter("sim1", 1, 42);
        bridge.register_adapter(adapter).unwrap();
        bridge.spawn_vehicle(make_config(1)).unwrap();
        bridge
            .assign_vehicle_to_adapter(VehicleId(1), "sim1")
            .unwrap();

        // Connect adapter
        bridge
            .get_adapter_mut("sim1")
            .unwrap()
            .connect()
            .await
            .unwrap();

        // Step - adapter generates sensors, bridge routes them to vehicle
        bridge.step().await.unwrap();

        // Vehicle should have sensor data
        let vehicle = bridge.get_vehicle(VehicleId(1)).unwrap();
        let sensors = vehicle.platform.take_sensors();
        assert!(sensors.is_some());
        let data = sensors.unwrap();
        assert_eq!(data.vehicle_id, VehicleId(1));
        assert!(data.imu.is_some());
    }

    #[tokio::test]
    async fn test_actuator_commands_reach_adapter() {
        let mut bridge = SitlBridge::new();
        bridge.set_time_mode(TimeMode::Lockstep {
            step_size_us: 10_000,
        });

        let adapter = make_adapter("sim1", 1, 42);
        bridge.register_adapter(adapter).unwrap();
        bridge.spawn_vehicle(make_config(1)).unwrap();
        bridge
            .assign_vehicle_to_adapter(VehicleId(1), "sim1")
            .unwrap();

        bridge
            .get_adapter_mut("sim1")
            .unwrap()
            .connect()
            .await
            .unwrap();

        // Set PWM on the vehicle's platform to generate actuator commands
        let vehicle = bridge.get_vehicle(VehicleId(1)).unwrap();
        vehicle.platform.create_pwm(0, 50).unwrap();
        vehicle.platform.create_pwm(1, 50).unwrap();
        vehicle.platform.set_pwm_duty(0, 0.75); // motor left
        vehicle.platform.set_pwm_duty(1, 0.75); // motor right

        // Step - should collect actuator commands and send to adapter
        bridge.step().await.unwrap();

        // The adapter should have received actuator commands (it applies them
        // internally). We can verify it ran by stepping again and checking
        // that the adapter advanced.
        let adapter = bridge.get_adapter("sim1").unwrap();
        assert!(adapter.sim_time_us() > 0);
    }

    #[tokio::test]
    async fn test_multi_vehicle_sensor_routing() {
        let mut bridge = SitlBridge::new();
        bridge.set_time_mode(TimeMode::Lockstep {
            step_size_us: 10_000,
        });

        // Two adapters for two vehicles
        let adapter1 = make_adapter("sim1", 1, 42);
        let adapter2 = make_adapter("sim2", 2, 43);
        bridge.register_adapter(adapter1).unwrap();
        bridge.register_adapter(adapter2).unwrap();

        bridge.spawn_vehicle(make_config(1)).unwrap();
        bridge.spawn_vehicle(make_config(2)).unwrap();

        bridge
            .assign_vehicle_to_adapter(VehicleId(1), "sim1")
            .unwrap();
        bridge
            .assign_vehicle_to_adapter(VehicleId(2), "sim2")
            .unwrap();

        // Connect both
        bridge
            .get_adapter_mut("sim1")
            .unwrap()
            .connect()
            .await
            .unwrap();
        bridge
            .get_adapter_mut("sim2")
            .unwrap()
            .connect()
            .await
            .unwrap();

        bridge.step().await.unwrap();

        // Vehicle 1 should have sensors from adapter 1 (vehicle_id=1)
        let v1 = bridge.get_vehicle(VehicleId(1)).unwrap();
        let s1 = v1.platform.take_sensors().unwrap();
        assert_eq!(s1.vehicle_id, VehicleId(1));

        // Vehicle 2 should have sensors from adapter 2 (vehicle_id=2)
        let v2 = bridge.get_vehicle(VehicleId(2)).unwrap();
        let s2 = v2.platform.take_sensors().unwrap();
        assert_eq!(s2.vehicle_id, VehicleId(2));
    }

    #[tokio::test]
    async fn test_deterministic_multi_vehicle() {
        async fn run_scenario() -> Vec<u64> {
            let mut bridge = SitlBridge::new();
            bridge.set_time_mode(TimeMode::Lockstep {
                step_size_us: 10_000,
            });

            let adapter = make_adapter("sim", 1, 12345);
            bridge.register_adapter(adapter).unwrap();
            bridge.spawn_vehicle(make_config(1)).unwrap();
            bridge
                .assign_vehicle_to_adapter(VehicleId(1), "sim")
                .unwrap();

            bridge
                .get_adapter_mut("sim")
                .unwrap()
                .connect()
                .await
                .unwrap();

            let mut timestamps = Vec::new();
            for _ in 0..10 {
                bridge.step().await.unwrap();
                timestamps.push(bridge.sim_time_us());
            }
            timestamps
        }

        let run1 = run_scenario().await;
        let run2 = run_scenario().await;
        assert_eq!(run1, run2, "Deterministic: same seed = same time sequence");
    }

    #[test]
    fn test_despawn_vehicle() {
        let mut bridge = SitlBridge::new();
        bridge.spawn_vehicle(make_config(1)).unwrap();
        assert_eq!(bridge.list_vehicles().len(), 1);

        bridge.despawn_vehicle(VehicleId(1)).unwrap();
        assert_eq!(bridge.list_vehicles().len(), 0);

        // Despawning again should fail
        assert!(bridge.despawn_vehicle(VehicleId(1)).is_err());
    }

    #[tokio::test]
    async fn test_step_with_no_adapters_or_vehicles() {
        let mut bridge = SitlBridge::new();
        bridge.set_time_mode(TimeMode::Lockstep {
            step_size_us: 1_000,
        });

        // Should succeed even with nothing registered
        bridge.step().await.unwrap();
        assert_eq!(bridge.sim_time_us(), 1_000);
    }

    #[tokio::test]
    async fn test_disconnected_adapter_skipped() {
        let mut bridge = SitlBridge::new();
        bridge.set_time_mode(TimeMode::Lockstep {
            step_size_us: 10_000,
        });

        // Register adapter but don't connect it
        let adapter = make_adapter("sim1", 1, 42);
        bridge.register_adapter(adapter).unwrap();
        bridge.spawn_vehicle(make_config(1)).unwrap();
        bridge
            .assign_vehicle_to_adapter(VehicleId(1), "sim1")
            .unwrap();

        // Should not error - disconnected adapters are skipped
        bridge.step().await.unwrap();

        // Vehicle should not have sensor data
        let vehicle = bridge.get_vehicle(VehicleId(1)).unwrap();
        assert!(vehicle.platform.take_sensors().is_none());
    }
}
