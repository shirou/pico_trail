//! MAVLink Parameter Protocol Handler
//!
//! Re-exports the core parameter handler and adds firmware-specific initialization
//! that loads parameters from Flash and registers all parameter groups.
//!
//! # Supported Messages
//!
//! - **PARAM_REQUEST_LIST**: Send all parameters to GCS (excludes hidden params)
//! - **PARAM_REQUEST_READ**: Send specific parameter by name or index (respects HIDDEN flag)
//! - **PARAM_SET**: Update parameter value with Flash persistence

pub use pico_trail_core::communication::handlers::param::{ParamHandler, ParamHandlerError};

use crate::parameters::ParamValue;
use crate::platform::traits::FlashInterface;

/// Extension trait for firmware-specific ParamHandler initialization
///
/// Provides `new_from_flash()` which loads parameters from Flash storage
/// and registers all parameter groups with defaults.
pub trait ParamHandlerInit {
    /// Create a new parameter handler with Flash persistence
    ///
    /// Loads parameters from Flash and registers defaults if needed.
    ///
    /// # Arguments
    ///
    /// * `flash` - Flash interface for parameter storage
    ///
    /// # Returns
    ///
    /// Returns a new parameter handler with default parameters registered.
    fn new_from_flash<F: FlashInterface>(flash: &mut F) -> Self;
}

impl ParamHandlerInit for ParamHandler {
    fn new_from_flash<F: FlashInterface>(flash: &mut F) -> Self {
        // Load parameters from Flash (or create empty store if no valid blocks)
        let mut store = crate::parameters::storage::load_from_flash(flash).unwrap_or_default();

        // Register WiFi parameters with defaults (only if not already loaded)
        let _ = crate::parameters::WifiParams::register_defaults(&mut store);

        // Register arming parameters with defaults
        let _ = crate::parameters::ArmingParams::register_defaults(&mut store);

        // Register battery parameters with defaults
        let _ = crate::parameters::BatteryParams::register_defaults(&mut store);

        // Register failsafe parameters with defaults
        let _ = crate::parameters::FailsafeParams::register_defaults(&mut store);

        // Register fence parameters with defaults
        let _ = crate::parameters::FenceParams::register_defaults(&mut store);

        // Register compass parameters with defaults (required by Mission Planner)
        let _ = crate::parameters::CompassParams::register_defaults(&mut store);

        // Register board pin parameters with hwdef defaults
        let _ = crate::parameters::BoardParams::register_defaults(&mut store);

        // Register navigation controller parameters
        let _ = crate::parameters::NavigationParams::register_defaults(&mut store);

        // Register default MAVLink stream rate parameters
        // SR_* parameters control telemetry stream rates (Hz)
        let _ = store.register(
            "SR_EXTRA1",
            ParamValue::Int(10),
            crate::parameters::storage::ParamFlags::empty(),
        );
        let _ = store.register(
            "SR_POSITION",
            ParamValue::Int(5),
            crate::parameters::storage::ParamFlags::empty(),
        );
        let _ = store.register(
            "SR_RC_CHAN",
            ParamValue::Int(5),
            crate::parameters::storage::ParamFlags::empty(),
        );
        let _ = store.register(
            "SR_RAW_SENS",
            ParamValue::Int(5),
            crate::parameters::storage::ParamFlags::empty(),
        );

        // System identification
        let _ = store.register(
            "SYSID_THISMAV",
            ParamValue::Int(1),
            crate::parameters::storage::ParamFlags::empty(),
        );

        ParamHandler::from_store(store)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;
    use mavlink::common::{
        MavMessage, MavParamType, PARAM_REQUEST_LIST_DATA, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA,
    };

    #[test]
    fn test_handler_creation() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new_from_flash(&mut flash);

        // Parameter count breakdown:
        // - WiFi: 6 (NET_SSID, NET_PASS are String type, excluded from MAVLink count)
        // - SR_*: 4 stream rate parameters
        // - SYSID_THISMAV: 1
        // - Arming: 5 (ARMING_CHECK, ARMING_OPTIONS, ARMING_REQUIRE, ARMING_ACCTHRESH, ARMING_RUDDER)
        // - Battery: 6 (BATT_ARM_VOLT, BATT_CRT_VOLT, BATT_FS_CRT_ACT, BATT_LOW_VOLT, BATT_CAPACITY, BATT_VOLT_MULT)
        // - Failsafe: 4 (FS_ACTION, FS_TIMEOUT, FS_GCS_TIMEOUT, FS_GCS_ENABLE)
        // - Fence: 2 (FENCE_AUTOENABLE, FENCE_ACTION)
        // - Compass: 4 (COMPASS_OFS_X, COMPASS_OFS_Y, COMPASS_OFS_Z, COMPASS_USE)
        // - Navigation: 12 (WP_RADIUS, WP_APPR_DIST, ATC_HDG_ERR, etc.)
        // - Board: 11 (PIN_M1_IN1..PIN_BATTERY_ADC)
        //   Note: Board pins only registered when pico2_w feature enabled
        // String type parameters (NET_SSID, NET_PASS) excluded from MAVLink
        // Hidden parameter (NET_PASS) further excluded from count()

        #[cfg(feature = "pico2_w")]
        {
            // With pico2_w: 6 + 4 + 1 + 5 + 6 + 4 + 2 + 4 + 12 + 11 = 55 total
            // Minus 1 hidden (NET_PASS) = 54 visible
            assert_eq!(handler.count(), 54);
        }

        #[cfg(not(feature = "pico2_w"))]
        {
            // Without pico2_w: 55 - 11 (Board) = 44 total
            // Minus 1 hidden (NET_PASS) = 43 visible
            assert_eq!(handler.count(), 43);
        }
    }

    #[test]
    fn test_request_list_excludes_hidden() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new_from_flash(&mut flash);

        let request = PARAM_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
        };

        let messages = handler.handle_request_list(&request);

        // Should return all non-String, non-hidden parameters
        // NET_SSID (String) and NET_PASS (String, hidden) cannot be sent via MAVLink
        // Sendable = count() minus non-hidden String params (NET_SSID)

        #[cfg(feature = "pico2_w")]
        assert_eq!(messages.len(), 53); // 54 visible - 1 String (NET_SSID)

        #[cfg(not(feature = "pico2_w"))]
        assert_eq!(messages.len(), 42); // 43 visible - 1 String (NET_SSID)

        // Verify NET_PASS is not in the list
        for msg in &messages {
            if let MavMessage::PARAM_VALUE(data) = msg {
                let name = core::str::from_utf8(&*data.param_id)
                    .unwrap()
                    .trim_end_matches('\0');
                assert_ne!(name, "NET_PASS", "NET_PASS should be hidden");
            }
        }
    }

    #[test]
    fn test_request_read_hidden_parameter() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new_from_flash(&mut flash);

        let mut param_id = [0u8; 16];
        param_id[..8].copy_from_slice(b"NET_PASS");

        let request = PARAM_REQUEST_READ_DATA {
            target_system: 1,
            target_component: 1,
            param_id: param_id.into(),
            param_index: -1,
        };

        // Should return None for hidden parameter
        let response = handler.handle_request_read(&request);
        assert!(response.is_none(), "NET_PASS should not be readable");
    }

    #[test]
    fn test_request_read_visible_parameter() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new_from_flash(&mut flash);

        // Use NET_DHCP instead of NET_SSID (String type not supported in MAVLink)
        let mut param_id = [0u8; 16];
        param_id[..8].copy_from_slice(b"NET_DHCP");

        let request = PARAM_REQUEST_READ_DATA {
            target_system: 1,
            target_component: 1,
            param_id: param_id.into(),
            param_index: -1,
        };

        let response = handler.handle_request_read(&request);
        assert!(response.is_some(), "NET_DHCP should be readable");
    }

    #[test]
    fn test_param_set() {
        let mut flash = MockFlash::new();
        let mut handler = ParamHandler::new_from_flash(&mut flash);

        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        // For INT32 type, encode the integer value as f32 bit pattern
        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id: param_id.into(),
            param_value: f32::from_bits(20u32), // Encode 20 as bit pattern
            param_type: MavParamType::MAV_PARAM_TYPE_INT32,
        };

        let result = handler.handle_set(&set_msg);
        assert!(result.is_ok());

        // Verify value changed
        assert_eq!(handler.store().get("SR_EXTRA1"), Some(&ParamValue::Int(20)));

        // Verify store is dirty
        assert!(handler.is_dirty());
    }

    #[test]
    fn test_param_set_hidden_parameter() {
        let mut flash = MockFlash::new();
        let mut handler = ParamHandler::new_from_flash(&mut flash);

        let mut param_id = [0u8; 16];
        param_id[..8].copy_from_slice(b"NET_PASS");

        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id: param_id.into(),
            param_value: 0.0, // String parameters not directly supported
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        };

        // For now, setting NET_PASS via PARAM_SET as float should succeed
        let result = handler.handle_set(&set_msg);
        assert!(result.is_ok(), "Should allow setting hidden parameter");
    }

    #[test]
    fn test_save_to_flash() {
        let mut flash = MockFlash::new();
        let mut handler = ParamHandler::new_from_flash(&mut flash);

        // Modify a parameter
        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        // For INT32 type, encode the integer value as f32 bit pattern
        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id: param_id.into(),
            param_value: f32::from_bits(25u32), // Encode 25 as bit pattern
            param_type: MavParamType::MAV_PARAM_TYPE_INT32,
        };

        handler.handle_set(&set_msg).unwrap();
        assert!(handler.is_dirty());

        // Save to Flash
        handler.save_to_flash(&mut flash).unwrap();
        assert!(!handler.is_dirty());
    }

    #[test]
    fn test_param_set_ipv4() {
        let mut flash = MockFlash::new();
        let mut handler = ParamHandler::new_from_flash(&mut flash);

        let mut param_id = [0u8; 16];
        param_id[..11].copy_from_slice(b"NET_NETMASK");

        // Set NET_NETMASK to 255.255.255.0
        // IPv4 [255, 255, 255, 0] as u32 big-endian = 0xFFFFFF00
        let ipv4_value = u32::from_be_bytes([255, 255, 255, 0]);
        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id: param_id.into(),
            param_value: f32::from_bits(ipv4_value), // Encode as bit pattern
            param_type: MavParamType::MAV_PARAM_TYPE_UINT32,
        };

        let result = handler.handle_set(&set_msg);
        assert!(result.is_ok());

        // Verify value stored as Ipv4, not Int
        assert_eq!(
            handler.store().get("NET_NETMASK"),
            Some(&ParamValue::Ipv4([255, 255, 255, 0]))
        );

        // Verify store is dirty
        assert!(handler.is_dirty());
    }
}
