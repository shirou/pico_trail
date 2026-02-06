pub mod capabilities;

use async_trait::async_trait;

pub use capabilities::{SensorCapabilities, SimulatorCapabilities};

use crate::error::SimulatorError;
use crate::types::{ActuatorCommands, SensorData};

/// Pluggable interface for different physics simulator backends.
///
/// Implementations must be `Send + Sync` for object safety, allowing
/// adapters to be stored as `Box<dyn SimulatorAdapter>`.
#[async_trait]
pub trait SimulatorAdapter: Send + Sync {
    /// Unique identifier for this adapter type (e.g., "gazebo", "lightweight").
    fn adapter_type(&self) -> &'static str;

    /// Human-readable name for this adapter instance.
    fn name(&self) -> &str;

    /// Connect to the simulator backend.
    async fn connect(&mut self) -> Result<(), SimulatorError>;

    /// Disconnect from the simulator backend.
    async fn disconnect(&mut self) -> Result<(), SimulatorError>;

    /// Check if currently connected.
    fn is_connected(&self) -> bool;

    /// Receive sensor data (non-blocking, returns `None` if no data available).
    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError>;

    /// Send actuator commands to the simulator.
    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError>;

    /// Request the simulator to advance one time step (lockstep mode).
    async fn step(&mut self) -> Result<(), SimulatorError>;

    /// Get current simulation time in microseconds.
    fn sim_time_us(&self) -> u64;

    /// Check if this simulator supports lockstep time synchronization.
    fn supports_lockstep(&self) -> bool;

    /// Get the capabilities of this simulator backend.
    fn capabilities(&self) -> SimulatorCapabilities;
}
