//! Rover control modes
//!
//! This module provides control mode implementations for rover vehicles.
//! Following ArduPilot's architecture, each mode implements the `Mode` trait
//! with enter/update/exit lifecycle hooks.
//!
//! ## Available Modes
//!
//! - **Manual**: Direct RC control (no stabilization)
//! - **Hold**: Stop in place
//! - **Auto**: Follow waypoint mission
//! - **RTL**: Return to launch
//! - **Guided**: Accept real-time commands from GCS
//!
//! ## References
//!
//! - ADR-w9zpl-control-mode-architecture: Trait-based mode architecture
//! - FR-sp3at-control-modes: Mode requirements
//! - ArduPilot Rover modes: https://ardupilot.org/rover/docs/rover-control-modes.html

pub mod circle;
pub mod loiter;
pub mod manual;

// Re-export mode implementations
pub use circle::{CircleConfig, CircleDirection, CircleMode};
#[cfg(feature = "rover")]
pub use loiter::{LoiterState, RoverLoiter};
pub use manual::ManualMode;

/// Control mode trait
///
/// All control modes (Manual, Hold, Auto, RTL, Guided) implement this trait.
///
/// ## Lifecycle
///
/// 1. `enter()` - Called once when entering the mode
/// 2. `update(dt)` - Called at 50 Hz while mode is active
/// 3. `exit()` - Called once when exiting the mode
///
/// ## Example
///
/// ```rust,ignore
/// pub struct ManualMode {
///     rc_input: &'static Mutex<RcInput>,
///     actuators: &'static mut dyn ActuatorInterface,
/// }
///
/// impl Mode for ManualMode {
///     fn enter(&mut self) -> Result<(), &'static str> {
///         defmt::info!("Entering Manual mode");
///         Ok(())
///     }
///
///     fn update(&mut self, dt: f32) -> Result<(), &'static str> {
///         // Read RC inputs and command actuators
///         Ok(())
///     }
///
///     fn exit(&mut self) -> Result<(), &'static str> {
///         defmt::info!("Exiting Manual mode");
///         self.actuators.set_steering(0.0)?;
///         self.actuators.set_throttle(0.0)?;
///         Ok(())
///     }
///
///     fn name(&self) -> &'static str {
///         "Manual"
///     }
/// }
/// ```
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
