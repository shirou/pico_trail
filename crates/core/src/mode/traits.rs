//! Mode trait definition
//!
//! Platform-agnostic interface for control mode state machines.

/// Control mode trait
///
/// All control modes (Manual, Hold, Auto, RTL, Guided) implement this trait.
///
/// # Lifecycle
///
/// 1. `enter()` - Called once when entering the mode
/// 2. `update(dt)` - Called at 50 Hz while mode is active
/// 3. `exit()` - Called once when exiting the mode
///
/// # Note
///
/// Platform-specific implementations (ActuatorInterface, Embassy mutex)
/// are provided by the firmware crate. This trait defines only the interface.
pub trait Mode {
    /// Initialize mode (called once on mode entry)
    ///
    /// Returns `Err` if mode cannot be entered (e.g., Auto without GPS).
    fn enter(&mut self) -> Result<(), &'static str>;

    /// Update mode (called at 50 Hz)
    ///
    /// # Arguments
    ///
    /// * `dt` - Delta time since last update (seconds)
    fn update(&mut self, dt: f32) -> Result<(), &'static str>;

    /// Cleanup mode (called once on mode exit)
    ///
    /// Should set actuators to safe state (neutral).
    fn exit(&mut self) -> Result<(), &'static str>;

    /// Get mode name for logging and telemetry
    fn name(&self) -> &'static str;
}
