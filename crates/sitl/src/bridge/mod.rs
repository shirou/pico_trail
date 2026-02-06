pub mod registry;
pub mod time;

use std::collections::HashMap;

pub use registry::AdapterRegistry;
pub use time::TimeMode;

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
    time_mode: TimeMode,
    sim_time_us: u64,
}

impl SitlBridge {
    pub fn new() -> Self {
        Self {
            registry: AdapterRegistry::new(),
            vehicles: HashMap::new(),
            time_mode: TimeMode::default(),
            sim_time_us: 0,
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
    pub fn spawn_vehicle(&mut self, config: VehicleConfig) -> Result<VehicleId, SimulatorError> {
        let id = config.id;
        if self.vehicles.contains_key(&id) {
            return Err(SimulatorError::VehicleAlreadyExists(id));
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

    // -- Simulation control --

    /// Advance the simulation by one step.
    ///
    /// In lockstep mode, advances all connected adapters by one time step.
    /// In free-running mode, polls all adapters for available data.
    pub async fn step(&mut self) -> Result<(), SimulatorError> {
        match &self.time_mode {
            TimeMode::Lockstep { step_size_us } => {
                self.sim_time_us += step_size_us;
            }
            TimeMode::FreeRunning | TimeMode::Scaled { .. } => {
                // Placeholder: in future phases, poll adapters for data
            }
        }
        Ok(())
    }

    /// Set the time synchronization mode.
    pub fn set_time_mode(&mut self, mode: TimeMode) {
        self.time_mode = mode;
    }

    /// Get the current simulation time in microseconds.
    pub fn sim_time_us(&self) -> u64 {
        self.sim_time_us
    }
}

impl Default for SitlBridge {
    fn default() -> Self {
        Self::new()
    }
}
