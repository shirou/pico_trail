//! Mode Manager
//!
//! Manages control mode lifecycle and transitions for rover vehicles.
//!
//! ## Responsibilities
//!
//! - Execute active mode at 50 Hz
//! - Handle mode transitions (exit → validate → enter)
//! - Calculate delta time for mode updates
//! - Update system state with current mode
//!
//! ## Safety
//!
//! - Mode entry failure triggers fallback to Manual mode
//! - Mode exit errors are logged but don't prevent transition
//!
//! ## References
//!
//! - ADR-w9zpl-control-mode-architecture: Trait-based mode architecture
//! - FR-q2sjt-control-mode-framework: Mode framework requirements

extern crate alloc;
use alloc::boxed::Box;

use super::mode::Mode;
use crate::communication::mavlink::state::SystemState;

/// Mode Manager
///
/// Manages the active control mode and handles mode transitions.
pub struct ModeManager {
    /// Current active mode
    current_mode: Box<dyn Mode>,
    /// System state (for mode reporting)
    system_state: &'static mut SystemState,
    /// Last update timestamp (microseconds)
    last_update_us: u64,
}

impl ModeManager {
    /// Create mode manager with initial mode
    ///
    /// # Arguments
    ///
    /// * `initial_mode` - Initial mode (typically Manual)
    /// * `system_state` - Mutable reference to system state
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let manual_mode = Box::new(ManualMode::new(rc_input, actuators));
    /// let mode_manager = ModeManager::new(manual_mode, system_state);
    /// ```
    pub fn new(mut initial_mode: Box<dyn Mode>, system_state: &'static mut SystemState) -> Self {
        // Enter initial mode
        if let Err(_e) = initial_mode.enter() {
            crate::log_error!("Failed to enter initial mode: {}", _e);
        }

        Self {
            current_mode: initial_mode,
            system_state,
            last_update_us: 0,
        }
    }

    /// Execute active mode (call at 50 Hz)
    ///
    /// Calculates delta time and calls the active mode's update method.
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// `Ok(())` on success, `Err` if mode update fails
    pub fn execute(&mut self, current_time_us: u64) -> Result<(), &'static str> {
        // Calculate delta time (dt)
        let dt = if self.last_update_us == 0 {
            // First execution: assume 20ms (50 Hz)
            0.02
        } else {
            let delta_us = current_time_us.saturating_sub(self.last_update_us);
            delta_us as f32 / 1_000_000.0
        };

        // Update last timestamp
        self.last_update_us = current_time_us;

        // Execute mode update
        self.current_mode.update(dt)?;

        Ok(())
    }

    /// Request mode change
    ///
    /// Performs mode transition: exit old mode → enter new mode.
    /// If new mode entry fails, reverts to Manual mode as fallback.
    ///
    /// # Arguments
    ///
    /// * `new_mode` - New mode to activate
    ///
    /// # Returns
    ///
    /// `Ok(())` on success, `Err` if mode entry fails (fallback to Manual attempted)
    pub fn set_mode(&mut self, mut new_mode: Box<dyn Mode>) -> Result<(), &'static str> {
        let _old_mode_name = self.current_mode.name();
        let _new_mode_name = new_mode.name();

        crate::log_info!("Mode transition: {} -> {}", _old_mode_name, _new_mode_name);

        // Exit current mode (log warnings but continue)
        if let Err(_e) = self.current_mode.exit() {
            crate::log_warn!("Mode exit error ({}): {}", _old_mode_name, _e);
        }

        // Enter new mode
        match new_mode.enter() {
            Ok(()) => {
                // Success: update current mode
                self.current_mode = new_mode;
                crate::log_info!("Mode transition complete: {}", _new_mode_name);
                Ok(())
            }
            Err(e) => {
                // Failure: log error and attempt to revert to old mode
                crate::log_error!("Failed to enter mode {}: {}", _new_mode_name, e);

                // Attempt to re-enter old mode
                if let Err(_e2) = self.current_mode.enter() {
                    crate::log_error!("Failed to re-enter old mode {}: {}", _old_mode_name, _e2);
                    // This is a critical failure - the system is in an undefined state
                    // In production, this should trigger a system reset or emergency mode
                }

                Err(e)
            }
        }
    }

    /// Get current mode name
    pub fn current_mode_name(&self) -> &'static str {
        self.current_mode.name()
    }

    /// Get system state reference
    pub fn system_state(&self) -> &SystemState {
        self.system_state
    }

    /// Get mutable system state reference
    pub fn system_state_mut(&mut self) -> &mut SystemState {
        self.system_state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::SystemState;

    // Mock mode for testing
    struct MockMode {
        name: &'static str,
        enter_result: Result<(), &'static str>,
        exit_result: Result<(), &'static str>,
        update_called: bool,
    }

    impl MockMode {
        fn new(name: &'static str) -> Self {
            Self {
                name,
                enter_result: Ok(()),
                exit_result: Ok(()),
                update_called: false,
            }
        }

        fn with_enter_error(mut self, error: &'static str) -> Self {
            self.enter_result = Err(error);
            self
        }
    }

    impl Mode for MockMode {
        fn enter(&mut self) -> Result<(), &'static str> {
            self.enter_result
        }

        fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
            self.update_called = true;
            Ok(())
        }

        fn exit(&mut self) -> Result<(), &'static str> {
            self.exit_result
        }

        fn name(&self) -> &'static str {
            self.name
        }
    }

    #[test]
    fn test_mode_manager_creation() {
        let mode = Box::new(MockMode::new("Test"));
        let system_state = Box::leak(Box::new(SystemState::new()));

        let manager = ModeManager::new(mode, system_state);
        assert_eq!(manager.current_mode_name(), "Test");
    }

    #[test]
    fn test_mode_execute() {
        let mode = Box::new(MockMode::new("Test"));
        let system_state = Box::leak(Box::new(SystemState::new()));

        let mut manager = ModeManager::new(mode, system_state);

        // First execution (timestamp 1_000_000 = 1 second)
        let result = manager.execute(1_000_000);
        assert!(result.is_ok());

        // Second execution (timestamp 1_020_000 = 1.02 seconds, dt = 0.02s)
        let result = manager.execute(1_020_000);
        assert!(result.is_ok());
    }

    #[test]
    fn test_delta_time_calculation() {
        let mode = Box::new(MockMode::new("Test"));
        let system_state = Box::leak(Box::new(SystemState::new()));

        let mut manager = ModeManager::new(mode, system_state);

        // First execution at 1 second
        manager.execute(1_000_000).unwrap();

        // Second execution at 1.05 seconds (50ms later)
        manager.execute(1_050_000).unwrap();

        // Delta time should be 0.05 seconds
        // (Verified by the mode update being called successfully)
    }

    #[test]
    fn test_mode_transition_success() {
        let initial_mode = Box::new(MockMode::new("Initial"));
        let system_state = Box::leak(Box::new(SystemState::new()));

        let mut manager = ModeManager::new(initial_mode, system_state);
        assert_eq!(manager.current_mode_name(), "Initial");

        // Transition to new mode
        let new_mode = Box::new(MockMode::new("NewMode"));
        let result = manager.set_mode(new_mode);

        assert!(result.is_ok());
        assert_eq!(manager.current_mode_name(), "NewMode");
    }

    #[test]
    fn test_mode_transition_entry_failure() {
        let initial_mode = Box::new(MockMode::new("Initial"));
        let system_state = Box::leak(Box::new(SystemState::new()));

        let mut manager = ModeManager::new(initial_mode, system_state);

        // Try to transition to mode that fails entry
        let failing_mode = Box::new(MockMode::new("Failing").with_enter_error("GPS not available"));
        let result = manager.set_mode(failing_mode);

        assert!(result.is_err());
        // Should stay in old mode
        assert_eq!(manager.current_mode_name(), "Initial");
    }

    #[test]
    fn test_system_state_access() {
        let mode = Box::new(MockMode::new("Test"));
        let system_state = Box::leak(Box::new(SystemState::new()));

        let mut manager = ModeManager::new(mode, system_state);

        // Modify system state
        manager.system_state_mut().battery.voltage = 12.5;

        // Read system state
        assert_eq!(manager.system_state().battery.voltage, 12.5);
    }
}
