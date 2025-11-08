//! Control Loop Task
//!
//! Embassy async task that executes the active control mode at 50 Hz.
//! This task is vehicle-agnostic and used by all vehicle types (Rover, Boat, Copter).
//!
//! ## Responsibilities
//!
//! - Execute mode manager at 50 Hz (20ms interval)
//! - Check RC timeout at each iteration
//! - Monitor and log mode execution errors
//! - Calculate timestamps for mode updates
//!
//! ## Integration
//!
//! This task requires:
//! - Mode manager with initial mode configured
//! - RC input state (global RC_INPUT mutex)
//! - System state for armed/disarmed status
//!
//! ## References
//!
//! - ADR-w9zpl-control-mode-architecture: Mode architecture
//! - FR-q2sjt-control-mode-framework: Mode framework requirements
//! - NFR-kqvyf-manual-control-latency: Latency requirements

#[cfg(feature = "pico2_w")]
use crate::libraries::RC_INPUT;
#[cfg(feature = "pico2_w")]
use crate::rover::ModeManager;
#[cfg(feature = "pico2_w")]
use embassy_time::{Duration, Instant, Ticker};

/// Control loop task (50 Hz mode execution)
///
/// This task executes the active control mode at 50 Hz (20ms interval).
/// It handles RC timeout detection and mode execution errors.
///
/// Vehicle-agnostic: Used by Rover, Boat, Copter, and other vehicle types.
///
/// # Arguments
///
/// * `mode_manager` - Mode manager with initial mode configured
///
/// # Example
///
/// ```rust,ignore
/// #[embassy_executor::main]
/// async fn main(spawner: Spawner) {
///     // Create mode manager with Manual mode
///     let manual_mode = Box::new(ManualMode::new(rc_input, actuators));
///     let mode_manager = ModeManager::new(manual_mode, system_state);
///
///     // Spawn control loop task
///     spawner.spawn(control_loop_task(mode_manager)).unwrap();
/// }
/// ```
#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
pub async fn control_loop_task(mut mode_manager: ModeManager) {
    crate::log_info!("Control loop task started");
    crate::log_info!("  Initial mode: {}", mode_manager.current_mode_name());

    // Create 50 Hz ticker (20ms interval)
    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        // Get current timestamp
        let current_time_us = Instant::now().as_micros();

        // Check RC timeout
        {
            let mut rc = RC_INPUT.lock().await;
            rc.check_timeout(current_time_us);

            // Log RC status changes
            if rc.is_lost() {
                crate::log_warn!("RC input lost (timeout)");
            }
        }

        // Execute mode manager
        match mode_manager.execute(current_time_us) {
            Ok(()) => {
                // Mode executed successfully
            }
            Err(e) => {
                // Log error but continue execution
                crate::log_error!(
                    "Mode execution error ({}): {}",
                    mode_manager.current_mode_name(),
                    e
                );
            }
        }

        // Wait for next tick
        ticker.next().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_control_task_compilation() {
        // This test just verifies that the task compiles correctly
        // Actual task execution requires Embassy runtime and cannot be tested in unit tests
    }
}
