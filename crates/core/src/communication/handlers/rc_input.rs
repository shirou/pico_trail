//! RC Input Handler
//!
//! Handles MAVLink RC input messages from ground control stations.
//!
//! ## Messages Handled
//!
//! - `RC_CHANNELS` (ID 65): RC channel values from receiver
//! - `RC_CHANNELS_OVERRIDE` (ID 70): RC channel override from GCS joystick/gamepad
//! - `MANUAL_CONTROL` (ID 69): Joystick axes from GCS

use crate::rc::RC_INPUT;
use crate::traits::sync::SharedState;

/// RC input message handler
///
/// Processes RC_CHANNELS and RC_CHANNELS_OVERRIDE messages to update the global RC state.
/// This handler updates the global RC_INPUT directly.
pub struct RcInputHandler {
    /// This vehicle's MAVLink system ID for target filtering.
    system_id: u8,
}

impl RcInputHandler {
    /// Create a new RC input handler for the given system ID.
    pub fn new() -> Self {
        Self { system_id: 1 }
    }

    /// Create a new RC input handler with a specific system ID.
    pub fn with_system_id(system_id: u8) -> Self {
        Self { system_id }
    }
}

impl Default for RcInputHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl RcInputHandler {
    /// Handle RC_CHANNELS message
    ///
    /// Updates RC input state with normalized channel values.
    pub async fn handle_rc_channels(
        &mut self,
        rc_data: &mavlink::common::RC_CHANNELS_DATA,
        current_time_us: u64,
    ) {
        let channels = [
            rc_data.chan1_raw,
            rc_data.chan2_raw,
            rc_data.chan3_raw,
            rc_data.chan4_raw,
            rc_data.chan5_raw,
            rc_data.chan6_raw,
            rc_data.chan7_raw,
            rc_data.chan8_raw,
            rc_data.chan9_raw,
            rc_data.chan10_raw,
            rc_data.chan11_raw,
            rc_data.chan12_raw,
            rc_data.chan13_raw,
            rc_data.chan14_raw,
            rc_data.chan15_raw,
            rc_data.chan16_raw,
            rc_data.chan17_raw,
            rc_data.chan18_raw,
        ];

        RC_INPUT.with_mut(|rc| {
            rc.update_from_mavlink(&channels, rc_data.chancount, current_time_us);
        });
    }

    /// Handle RC_CHANNELS_OVERRIDE message
    ///
    /// Updates RC input state with override channel values from GCS joystick/gamepad.
    pub async fn handle_rc_channels_override(
        &mut self,
        rc_override: &mavlink::common::RC_CHANNELS_OVERRIDE_DATA,
        current_time_us: u64,
    ) -> bool {
        if rc_override.target_system != self.system_id && rc_override.target_system != 0 {
            return false;
        }

        let channels = [
            rc_override.chan1_raw,
            rc_override.chan2_raw,
            rc_override.chan3_raw,
            rc_override.chan4_raw,
            rc_override.chan5_raw,
            rc_override.chan6_raw,
            rc_override.chan7_raw,
            rc_override.chan8_raw,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        RC_INPUT.with_mut(|rc| {
            rc.update_from_pwm(&channels, 8, current_time_us);
        });

        true
    }

    /// Handle MANUAL_CONTROL message
    ///
    /// Maps joystick axes to RC channels:
    /// - `y` → channel 1 (steering): left/right [-1000, 1000]
    /// - `x` → channel 3 (throttle): forward/back [-1000, 1000]
    ///
    /// Axes with value `INT16_MAX` (32767) are ignored (axis invalid).
    /// Values are converted from [-1000, 1000] to PWM range [1000, 2000].
    pub async fn handle_manual_control(
        &mut self,
        data: &mavlink::common::MANUAL_CONTROL_DATA,
        current_time_us: u64,
    ) -> bool {
        // Validate target (accept 0 = broadcast or own system_id)
        if data.target != 0 && data.target != self.system_id {
            return false;
        }

        const IGNORE: i16 = i16::MAX; // 32767

        // Convert [-1000, 1000] to PWM [1000, 2000]: pwm = (axis + 1000) / 2 + 1000
        let steering_pwm = if data.y == IGNORE {
            1500 // neutral
        } else {
            ((data.y as i32 + 1000) / 2 + 1000).clamp(1000, 2000) as u16
        };

        // Throttle channel uses inverted normalization (1000 = forward = +1.0).
        // MANUAL_CONTROL x=+1000 means forward, so map to PWM 1000.
        // Formula: pwm = 1500 - x/2
        let throttle_pwm = if data.x == IGNORE {
            1500 // neutral
        } else {
            (1500 - data.x as i32 / 2).clamp(1000, 2000) as u16
        };

        let channels = [
            steering_pwm, // ch1: steering
            1500,         // ch2: unused
            throttle_pwm, // ch3: throttle
            1500,         // ch4: unused
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        RC_INPUT.with_mut(|rc| {
            rc.update_from_pwm(&channels, 4, current_time_us);
        });

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manual_control_converts_axes_to_pwm() {
        let mut handler = RcInputHandler::new();
        let data = mavlink::common::MANUAL_CONTROL_DATA {
            target: 1,
            x: 500,   // throttle: 500 → PWM (500+1000)/2+1000 = 1750
            y: -1000, // steering full left → PWM (-1000+1000)/2+1000 = 1000
            z: 0,
            r: 0,
            buttons: 0,
            buttons2: 0,
            enabled_extensions: 0,
            s: 0,
            t: 0,
            aux1: 0,
            aux2: 0,
            aux3: 0,
            aux4: 0,
            aux5: 0,
            aux6: 0,
        };

        embassy_futures::block_on(async {
            let result = handler.handle_manual_control(&data, 1_000_000).await;
            assert!(result);
        });

        // Check channel 1 (steering) and channel 3 (throttle)
        let (ch1, ch3) = RC_INPUT.with(|rc| (rc.get_channel(1), rc.get_channel(3)));
        // ch1: PWM 1000 → normalized -1.0
        assert!(
            (ch1 - (-1.0)).abs() < 0.01,
            "steering should be -1.0, got {ch1}"
        );
        // ch3: PWM 1750 → normalized 0.5
        assert!(
            (ch3 - 0.5).abs() < 0.01,
            "throttle should be 0.5, got {ch3}"
        );
    }

    #[test]
    fn test_manual_control_ignores_invalid_axes() {
        let mut handler = RcInputHandler::new();
        let data = mavlink::common::MANUAL_CONTROL_DATA {
            target: 1,
            x: i16::MAX, // ignore throttle
            y: 0,        // steering center → PWM 1500
            z: 0,
            r: 0,
            buttons: 0,
            buttons2: 0,
            enabled_extensions: 0,
            s: 0,
            t: 0,
            aux1: 0,
            aux2: 0,
            aux3: 0,
            aux4: 0,
            aux5: 0,
            aux6: 0,
        };

        embassy_futures::block_on(async {
            let result = handler.handle_manual_control(&data, 2_000_000).await;
            assert!(result);
        });

        // ch3 should be neutral (1500 PWM → 0.0)
        let ch3 = RC_INPUT.with(|rc| rc.get_channel(3));
        assert!(
            ch3.abs() < 0.01,
            "ignored axis should be neutral, got {ch3}"
        );
    }

    #[test]
    fn test_manual_control_rejects_wrong_target() {
        let mut handler = RcInputHandler::new();
        let data = mavlink::common::MANUAL_CONTROL_DATA {
            target: 99,
            x: 500,
            y: 500,
            z: 0,
            r: 0,
            buttons: 0,
            buttons2: 0,
            enabled_extensions: 0,
            s: 0,
            t: 0,
            aux1: 0,
            aux2: 0,
            aux3: 0,
            aux4: 0,
            aux5: 0,
            aux6: 0,
        };

        embassy_futures::block_on(async {
            let result = handler.handle_manual_control(&data, 3_000_000).await;
            assert!(!result, "Should reject target=99");
        });
    }
}
