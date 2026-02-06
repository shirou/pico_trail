use crate::types::VehicleId;

/// Errors that can occur during simulator operations.
#[derive(Debug, thiserror::Error)]
pub enum SimulatorError {
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Protocol error: {0}")]
    ProtocolError(String),

    #[error("Timeout waiting for {0}")]
    Timeout(&'static str),

    #[error("Adapter not found: {0}")]
    AdapterNotFound(String),

    #[error("Adapter already registered: {0}")]
    AdapterAlreadyRegistered(String),

    #[error("Vehicle not found: {0}")]
    VehicleNotFound(VehicleId),

    #[error("Vehicle already exists: {0}")]
    VehicleAlreadyExists(VehicleId),

    #[error("Vehicle not assigned to adapter: {0}")]
    VehicleNotAssigned(VehicleId),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}
