//! MAVLink Parameter Protocol Handler
//!
//! Implements parameter read/write protocol for Ground Control Stations.
//!
//! # Supported Messages
//!
//! - **PARAM_REQUEST_LIST**: Send all parameters to GCS (excludes hidden params)
//! - **PARAM_REQUEST_READ**: Send specific parameter by name or index (respects HIDDEN flag)
//! - **PARAM_SET**: Update parameter value with Flash persistence
//!
//! # Parameter Storage
//!
//! Parameters are stored in Flash-backed ParameterStore with CRC validation.
//! See `src/parameters/storage.rs` for implementation details.
//!
//! # Hidden Parameters
//!
//! Parameters marked with `ParamFlags::HIDDEN` (e.g., NET_PASS) are:
//! - Not visible in PARAM_REQUEST_LIST
//! - Return empty response for PARAM_REQUEST_READ
//! - Can still be SET via PARAM_SET (for initial configuration)
//!
//! # Flash Persistence
//!
//! Parameter changes are marked dirty in ParameterStore. Caller is responsible
//! for calling `save_to_flash()` to persist changes.

use crate::parameters::{ParamValue, ParameterStore};
use crate::platform::traits::FlashInterface;
use mavlink::common::{
    MavMessage, MavParamType, PARAM_REQUEST_LIST_DATA, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA,
    PARAM_VALUE_DATA,
};

/// Parameter handler error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamHandlerError {
    /// Parameter not found
    NotFound,
    /// Invalid parameter value
    InvalidValue,
    /// Parameter is read-only
    ReadOnly,
    /// Store error
    StoreError,
}

/// Parameter protocol handler
///
/// Handles PARAM_* messages from GCS and manages parameter store.
#[derive(Default)]
pub struct ParamHandler {
    /// Parameter store with Flash persistence
    store: ParameterStore,
}

impl ParamHandler {
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
    pub fn new<F: FlashInterface>(flash: &mut F) -> Self {
        // Load parameters from Flash (or create empty store if no valid blocks)
        let mut store = ParameterStore::load_from_flash(flash).unwrap_or_default();

        // Register WiFi parameters with defaults (only if not already loaded)
        let _ = crate::parameters::WifiParams::register_defaults(&mut store);

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

        Self { store }
    }

    /// Get parameter count (excluding hidden parameters)
    pub fn count(&self) -> usize {
        self.store.count()
    }

    /// Handle PARAM_REQUEST_LIST message
    ///
    /// Returns a vector of PARAM_VALUE messages for all non-hidden parameters.
    ///
    /// # Arguments
    ///
    /// * `_data` - PARAM_REQUEST_LIST message data (unused, requests all params)
    ///
    /// # Returns
    ///
    /// Vector of PARAM_VALUE messages to send to GCS.
    pub fn handle_request_list(
        &self,
        _data: &PARAM_REQUEST_LIST_DATA,
    ) -> heapless::Vec<MavMessage, 64> {
        let mut messages = heapless::Vec::new();
        let count = self.store.count(); // Excludes hidden parameters

        let mut index: u16 = 0;
        for name in self.store.iter_names() {
            if let Some(value) = self.store.get(name.as_str()) {
                if let Some(msg) =
                    self.create_param_value_message(name.as_str(), value, index, count as u16)
                {
                    let _ = messages.push(msg);
                    index += 1;
                    if messages.is_full() {
                        break; // Limit batch size
                    }
                }
            }
        }

        messages
    }

    /// Handle PARAM_REQUEST_READ message
    ///
    /// Returns a PARAM_VALUE message for the requested parameter.
    /// Hidden parameters return None (not readable).
    ///
    /// # Arguments
    ///
    /// * `data` - PARAM_REQUEST_READ message data
    ///
    /// # Returns
    ///
    /// PARAM_VALUE message if parameter found and not hidden, or None otherwise.
    pub fn handle_request_read(&self, data: &PARAM_REQUEST_READ_DATA) -> Option<MavMessage> {
        // Try by index first (only non-hidden parameters)
        if data.param_index >= 0 {
            let index = data.param_index as usize;
            for (current_index, name) in self.store.iter_names().enumerate() {
                if current_index == index {
                    if let Some(value) = self.store.get(name.as_str()) {
                        return self.create_param_value_message(
                            name.as_str(),
                            value,
                            index as u16,
                            self.store.count() as u16,
                        );
                    }
                }
            }
            return None;
        }

        // Try by name
        let param_id = core::str::from_utf8(&data.param_id)
            .ok()?
            .trim_end_matches('\0');

        // Check if parameter is hidden (return None if hidden)
        if self.store.is_hidden(param_id) {
            return None;
        }

        if let Some(value) = self.store.get(param_id) {
            // Find index for response
            for (index, name) in (0_u16..).zip(self.store.iter_names()) {
                if name.as_str() == param_id {
                    return self.create_param_value_message(
                        param_id,
                        value,
                        index,
                        self.store.count() as u16,
                    );
                }
            }
        }

        None
    }

    /// Handle PARAM_SET message
    ///
    /// Validates and sets parameter value, marks store as dirty.
    /// Caller is responsible for saving to Flash.
    ///
    /// # Arguments
    ///
    /// * `data` - PARAM_SET message data
    ///
    /// # Returns
    ///
    /// PARAM_VALUE message with updated value on success.
    pub fn handle_set(&mut self, data: &PARAM_SET_DATA) -> Result<MavMessage, ParamHandlerError> {
        let param_id = core::str::from_utf8(&data.param_id)
            .map_err(|_| ParamHandlerError::NotFound)?
            .trim_end_matches('\0');

        // Convert MAVLink value to ParamValue based on type
        let value = match data.param_type {
            MavParamType::MAV_PARAM_TYPE_REAL32 => ParamValue::Float(data.param_value),
            MavParamType::MAV_PARAM_TYPE_UINT32 | MavParamType::MAV_PARAM_TYPE_INT32 => {
                ParamValue::Int(data.param_value as i32)
            }
            MavParamType::MAV_PARAM_TYPE_UINT8 => ParamValue::Int(data.param_value as i32),
            _ => return Err(ParamHandlerError::InvalidValue),
        };

        // Set parameter (marks store as dirty)
        self.store
            .set(param_id, value)
            .map_err(|_| ParamHandlerError::StoreError)?;

        // Return updated PARAM_VALUE
        let updated_value = self
            .store
            .get(param_id)
            .ok_or(ParamHandlerError::NotFound)?;

        // Find index for response (only count non-hidden parameters)
        for (index, name) in (0_u16..).zip(self.store.iter_names()) {
            if name.as_str() == param_id {
                return self
                    .create_param_value_message(
                        param_id,
                        updated_value,
                        index,
                        self.store.count() as u16,
                    )
                    .ok_or(ParamHandlerError::StoreError);
            }
        }

        // If we get here, parameter was hidden or not found in iter_names
        // For hidden parameters, return a response anyway (SET is allowed)
        if self.store.is_hidden(param_id) {
            return self
                .create_param_value_message(param_id, updated_value, 0, self.store.count() as u16)
                .ok_or(ParamHandlerError::StoreError);
        }

        Err(ParamHandlerError::NotFound)
    }

    /// Create PARAM_VALUE message from parameter name and value
    fn create_param_value_message(
        &self,
        name: &str,
        value: &ParamValue,
        index: u16,
        count: u16,
    ) -> Option<MavMessage> {
        // Convert parameter name to fixed-size array
        let mut param_id = [0u8; 16];
        let name_bytes = name.as_bytes();
        let copy_len = name_bytes.len().min(16);
        param_id[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        // Convert value to f32 and type for MAVLink
        let (param_value, param_type) = match value {
            ParamValue::Float(f) => (*f, MavParamType::MAV_PARAM_TYPE_REAL32),
            ParamValue::Int(i) => (*i as f32, MavParamType::MAV_PARAM_TYPE_INT32),
            ParamValue::Bool(b) => (
                if *b { 1.0 } else { 0.0 },
                MavParamType::MAV_PARAM_TYPE_UINT8,
            ),
            ParamValue::Ipv4(ip) => {
                // Encode IPv4 as uint32 (big-endian)
                let value_u32 = u32::from_be_bytes(*ip);
                (value_u32 as f32, MavParamType::MAV_PARAM_TYPE_UINT32)
            }
            ParamValue::String(_) => {
                // MAVLink doesn't support string parameters directly
                // Return as float 0.0 (or skip)
                return None;
            }
        };

        Some(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value,
            param_count: count,
            param_index: index,
            param_id,
            param_type,
        }))
    }

    /// Get reference to parameter store
    pub fn store(&self) -> &ParameterStore {
        &self.store
    }

    /// Get mutable reference to parameter store
    pub fn store_mut(&mut self) -> &mut ParameterStore {
        &mut self.store
    }

    /// Check if store has unsaved changes
    pub fn is_dirty(&self) -> bool {
        self.store.is_dirty()
    }

    /// Save parameters to Flash
    ///
    /// # Arguments
    ///
    /// * `flash` - Flash interface
    ///
    /// # Returns
    ///
    /// Ok if saved successfully
    pub fn save_to_flash<F: FlashInterface>(
        &mut self,
        flash: &mut F,
    ) -> Result<(), ParamHandlerError> {
        self.store
            .save_to_flash(flash)
            .map_err(|_| ParamHandlerError::StoreError)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;

    #[test]
    fn test_handler_creation() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new(&mut flash);

        // Should have WiFi parameters (6) + SR parameters (4) + SYSID_THISMAV (1) = 11 total
        // NET_PASS is hidden, so count() excludes it: 10 visible parameters
        // Note: NET_SSID and NET_PASS are String type, cannot be sent via MAVLink PARAM_VALUE
        assert_eq!(handler.count(), 10);
    }

    #[test]
    fn test_request_list_excludes_hidden() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new(&mut flash);

        let request = PARAM_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
        };

        let messages = handler.handle_request_list(&request);

        // Should return all non-String parameters except NET_PASS (hidden)
        // NET_SSID (String) and NET_PASS (String, hidden) cannot be sent via MAVLink
        // So we get: 4 WiFi params (DHCP, IP, NETMASK, GATEWAY) + 4 SR params + 1 SYSID = 9
        assert_eq!(messages.len(), 9);

        // Verify NET_PASS is not in the list
        for msg in &messages {
            if let MavMessage::PARAM_VALUE(data) = msg {
                let name = core::str::from_utf8(&data.param_id)
                    .unwrap()
                    .trim_end_matches('\0');
                assert_ne!(name, "NET_PASS", "NET_PASS should be hidden");
            }
        }
    }

    #[test]
    fn test_request_read_hidden_parameter() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new(&mut flash);

        let mut param_id = [0u8; 16];
        param_id[..8].copy_from_slice(b"NET_PASS");

        let request = PARAM_REQUEST_READ_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_index: -1,
        };

        // Should return None for hidden parameter
        let response = handler.handle_request_read(&request);
        assert!(response.is_none(), "NET_PASS should not be readable");
    }

    #[test]
    fn test_request_read_visible_parameter() {
        let mut flash = MockFlash::new();
        let handler = ParamHandler::new(&mut flash);

        // Use NET_DHCP instead of NET_SSID (String type not supported in MAVLink)
        let mut param_id = [0u8; 16];
        param_id[..8].copy_from_slice(b"NET_DHCP");

        let request = PARAM_REQUEST_READ_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_index: -1,
        };

        let response = handler.handle_request_read(&request);
        assert!(response.is_some(), "NET_DHCP should be readable");
    }

    #[test]
    fn test_param_set() {
        let mut flash = MockFlash::new();
        let mut handler = ParamHandler::new(&mut flash);

        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_value: 20.0,
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
        let mut handler = ParamHandler::new(&mut flash);

        let mut param_id = [0u8; 16];
        param_id[..8].copy_from_slice(b"NET_PASS");

        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_value: 0.0, // String parameters not directly supported
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        };

        // This will fail because we're trying to set a String parameter with a Float value
        // In practice, GCS would need to use a different mechanism for string parameters
        // or we'd need to extend the protocol

        // For now, setting NET_PASS via PARAM_SET as float should succeed
        // (it will be stored as Float(0.0), but this is a limitation of MAVLink)
        let result = handler.handle_set(&set_msg);
        assert!(result.is_ok(), "Should allow setting hidden parameter");
    }

    #[test]
    fn test_save_to_flash() {
        let mut flash = MockFlash::new();
        let mut handler = ParamHandler::new(&mut flash);

        // Modify a parameter
        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_value: 25.0,
            param_type: MavParamType::MAV_PARAM_TYPE_INT32,
        };

        handler.handle_set(&set_msg).unwrap();
        assert!(handler.is_dirty());

        // Save to Flash
        handler.save_to_flash(&mut flash).unwrap();
        assert!(!handler.is_dirty());
    }
}
