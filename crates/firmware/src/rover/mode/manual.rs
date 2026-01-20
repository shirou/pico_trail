//! Manual Mode
//!
//! Direct RC control with no stabilization or autonomous behavior.
//!
//! ## Behavior
//!
//! - Read RC inputs (channel 1 = steering, channel 3 = throttle)
//! - Pass through directly to actuators
//! - Fail-safe to neutral on RC timeout
//! - Actuators neutral when disarmed (enforced by actuator layer)
//!
//! ## Safety
//!
//! - Multi-layer safety:
//!   1. RC timeout detection (1 second) → neutral outputs
//!   2. Armed state check (actuator layer) → neutral when disarmed
//! - No autonomous behavior - pilot in full control
//!
//! ## References
//!
//! - FR-uk0us-manual-mode: Manual mode requirements
//! - ADR-w9zpl-control-mode-architecture: Mode architecture
//! - ArduPilot Manual Mode: https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_manual.cpp

use crate::libraries::ActuatorInterface;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use super::Mode;

/// Manual Mode
///
/// Provides direct RC control with no stabilization.
pub struct ManualMode<'a> {
    rc_input: &'static Mutex<CriticalSectionRawMutex, crate::libraries::RcInput>,
    actuators: &'a mut dyn ActuatorInterface,
}

impl<'a> ManualMode<'a> {
    /// Create new Manual mode
    ///
    /// # Arguments
    ///
    /// * `rc_input` - RC input state (global RC_INPUT)
    /// * `actuators` - Actuator interface for steering and throttle
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let manual_mode = ManualMode::new(&RC_INPUT, actuators);
    /// ```
    pub fn new(
        rc_input: &'static Mutex<CriticalSectionRawMutex, crate::libraries::RcInput>,
        actuators: &'a mut dyn ActuatorInterface,
    ) -> Self {
        Self {
            rc_input,
            actuators,
        }
    }
}

impl<'a> Mode for ManualMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Entering Manual mode");
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // This function is async because we need to lock the RC input mutex
        // Embassy's async runtime will handle this
        embassy_futures::block_on(self.update_async())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Manual mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Manual"
    }
}

impl<'a> ManualMode<'a> {
    /// Async update implementation (embedded only)
    ///
    /// This is separated from update() to handle the async RC input lock.
    async fn update_async(&mut self) -> Result<(), &'static str> {
        // Lock RC input briefly
        let rc = self.rc_input.lock().await;

        // Check RC timeout
        if rc.is_lost() {
            // Fail-safe: neutral outputs
            crate::log_warn!("RC lost in Manual mode, neutral outputs");
            drop(rc); // Release lock before calling actuators
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Read RC channels (1-indexed)
        let steering = rc.get_channel(1); // Channel 1: steering
        let throttle = rc.get_channel(3); // Channel 3: throttle
        drop(rc); // Release lock

        // Direct pass-through to actuators
        // Note: Actuator layer enforces armed check
        self.actuators.set_steering(steering)?;
        self.actuators.set_throttle(throttle)?;

        Ok(())
    }
}
