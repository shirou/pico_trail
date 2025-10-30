//! MAVLink Parameter Protocol Handler
//!
//! Implements parameter read/write protocol for Ground Control Stations.
//!
//! # Supported Messages
//!
//! - **PARAM_REQUEST_LIST**: Send all parameters to GCS
//! - **PARAM_REQUEST_READ**: Send specific parameter by name or index
//! - **PARAM_SET**: Update parameter value with validation
//!
//! # Parameter Storage
//!
//! Parameters are stored in a ParameterRegistry with Flash persistence.
//! See `src/core/parameters/` for implementation details.

use crate::core::parameters::{
    ParamMetadata, ParamType, ParamValue, ParameterRegistry, RegistryError,
};
use crate::platform::traits::flash::FlashInterface;
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
    /// Registry error
    RegistryError,
}

impl From<RegistryError> for ParamHandlerError {
    fn from(err: RegistryError) -> Self {
        match err {
            RegistryError::NotFound => ParamHandlerError::NotFound,
            RegistryError::InvalidValue => ParamHandlerError::InvalidValue,
            _ => ParamHandlerError::RegistryError,
        }
    }
}

/// Parameter protocol handler
///
/// Handles PARAM_* messages from GCS and manages parameter registry.
pub struct ParamHandler<F: FlashInterface> {
    /// Parameter registry
    registry: ParameterRegistry<F>,
}

impl<F: FlashInterface> ParamHandler<F> {
    /// Create a new parameter handler with Flash persistence
    ///
    /// # Arguments
    ///
    /// * `flash` - Flash interface for parameter storage
    ///
    /// # Returns
    ///
    /// Returns a new parameter handler with default MAVLink parameters registered.
    pub fn new(flash: F) -> Self {
        let mut registry = ParameterRegistry::with_flash(flash);

        // Register default MAVLink stream rate parameters
        // SR_* parameters control telemetry stream rates (Hz)
        let _ = registry.register(ParamMetadata::new_uint32("SR_EXTRA1", 10, 0, 50));
        let _ = registry.register(ParamMetadata::new_uint32("SR_POSITION", 5, 0, 50));
        let _ = registry.register(ParamMetadata::new_uint32("SR_RC_CHAN", 5, 0, 50));
        let _ = registry.register(ParamMetadata::new_uint32("SR_RAW_SENS", 5, 0, 50));

        // System identification
        let _ = registry.register(ParamMetadata::new_uint32("SYSID_THISMAV", 1, 1, 255));

        // Load parameters from Flash if available
        let _ = registry.load_from_flash();

        Self { registry }
    }

    /// Create a new parameter handler without Flash persistence (RAM only)
    pub fn new_ram_only() -> Self
    where
        F: Default,
    {
        Self::new(F::default())
    }

    /// Get parameter count
    pub fn count(&self) -> usize {
        self.registry.count()
    }

    /// Handle PARAM_REQUEST_LIST message
    ///
    /// Returns a vector of PARAM_VALUE messages for all parameters.
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
    ) -> heapless::Vec<MavMessage, 16> {
        let mut messages = heapless::Vec::new();
        let count = self.registry.count();

        for index in 0..count {
            if let Some(param) = self.registry.get_by_index(index) {
                if let Some(msg) =
                    self.create_param_value_message(param, index as u16, count as u16)
                {
                    let _ = messages.push(msg);
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
    ///
    /// # Arguments
    ///
    /// * `data` - PARAM_REQUEST_READ message data
    ///
    /// # Returns
    ///
    /// PARAM_VALUE message if parameter found, or None if not found.
    pub fn handle_request_read(&self, data: &PARAM_REQUEST_READ_DATA) -> Option<MavMessage> {
        // Try by index first
        if data.param_index >= 0 {
            let index = data.param_index as usize;
            if let Some(param) = self.registry.get_by_index(index) {
                let count = self.registry.count();
                return self.create_param_value_message(param, index as u16, count as u16);
            }
        }

        // Try by name
        let param_id = core::str::from_utf8(&data.param_id)
            .ok()?
            .trim_end_matches('\0');

        if let Some(param) = self.registry.get_by_name(param_id) {
            // Find index for response
            let count = self.registry.count();
            for index in 0..count {
                if let Some(p) = self.registry.get_by_index(index) {
                    if p.name == param.name {
                        return self.create_param_value_message(param, index as u16, count as u16);
                    }
                }
            }
        }

        None
    }

    /// Handle PARAM_SET message
    ///
    /// Validates and sets parameter value, returns PARAM_VALUE response.
    ///
    /// # Arguments
    ///
    /// * `data` - PARAM_SET message data
    ///
    /// # Returns
    ///
    /// PARAM_VALUE message with updated value on success, or current value if validation failed.
    pub fn handle_set(&mut self, data: &PARAM_SET_DATA) -> Result<MavMessage, ParamHandlerError> {
        let param_id = core::str::from_utf8(&data.param_id)
            .map_err(|_| ParamHandlerError::NotFound)?
            .trim_end_matches('\0');

        // Convert MAVLink param type to internal type
        let param_type = match data.param_type {
            MavParamType::MAV_PARAM_TYPE_REAL32 => ParamType::Float,
            MavParamType::MAV_PARAM_TYPE_UINT32 => ParamType::Uint32,
            MavParamType::MAV_PARAM_TYPE_INT32 => ParamType::Uint32, // Treat as Uint32
            MavParamType::MAV_PARAM_TYPE_UINT8 => ParamType::Uint32, // Treat as Uint32
            _ => return Err(ParamHandlerError::InvalidValue),
        };

        // Convert value
        let value = match param_type {
            ParamType::Float => ParamValue::Float(data.param_value),
            ParamType::Uint32 => ParamValue::Uint32(data.param_value as u32),
        };

        // Set parameter
        self.registry.set_by_name(param_id, value)?;

        // Trigger save if Flash is available (will be handled by ParamSaver task)
        // For Phase 2, just set modified flag (save will be manual or deferred)

        // Return updated PARAM_VALUE
        let param = self
            .registry
            .get_by_name(param_id)
            .ok_or(ParamHandlerError::NotFound)?;

        // Find index for response
        let count = self.registry.count();
        for index in 0..count {
            if let Some(p) = self.registry.get_by_index(index) {
                if p.name == param.name {
                    return self
                        .create_param_value_message(param, index as u16, count as u16)
                        .ok_or(ParamHandlerError::RegistryError);
                }
            }
        }

        Err(ParamHandlerError::NotFound)
    }

    /// Create PARAM_VALUE message from parameter metadata
    fn create_param_value_message(
        &self,
        param: &ParamMetadata,
        index: u16,
        count: u16,
    ) -> Option<MavMessage> {
        // Convert parameter name to fixed-size array
        let mut param_id = [0u8; 16];
        let name_bytes = param.name.as_bytes();
        let copy_len = name_bytes.len().min(16);
        param_id[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        // Convert value to f32 for MAVLink
        let param_value = match param.value {
            ParamValue::Float(f) => f,
            ParamValue::Uint32(u) => u as f32,
        };

        // Convert type to MAVLink type
        let param_type = match param.param_type {
            ParamType::Float => MavParamType::MAV_PARAM_TYPE_REAL32,
            ParamType::Uint32 => MavParamType::MAV_PARAM_TYPE_UINT32,
        };

        Some(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value,
            param_count: count,
            param_index: index,
            param_id,
            param_type,
        }))
    }

    /// Get reference to parameter registry
    pub fn registry(&self) -> &ParameterRegistry<F> {
        &self.registry
    }

    /// Get mutable reference to parameter registry
    pub fn registry_mut(&mut self) -> &mut ParameterRegistry<F> {
        &mut self.registry
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;

    #[test]
    fn test_handler_creation() {
        let flash = MockFlash::new();
        let handler = ParamHandler::new(flash);

        // Should have 5 default parameters registered
        assert_eq!(handler.count(), 5);
    }

    #[test]
    fn test_default_parameters() {
        let flash = MockFlash::new();
        let handler = ParamHandler::new(flash);

        // Check SR_EXTRA1
        let param = handler.registry().get_by_name("SR_EXTRA1").unwrap();
        assert_eq!(param.value, ParamValue::Uint32(10));

        // Check SYSID_THISMAV
        let param = handler.registry().get_by_name("SYSID_THISMAV").unwrap();
        assert_eq!(param.value, ParamValue::Uint32(1));
    }

    #[test]
    fn test_request_list() {
        let flash = MockFlash::new();
        let handler = ParamHandler::new(flash);

        let request = PARAM_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
        };

        let messages = handler.handle_request_list(&request);
        assert_eq!(messages.len(), 5); // Should return all 5 parameters

        // Verify first message
        if let MavMessage::PARAM_VALUE(data) = &messages[0] {
            assert_eq!(data.param_count, 5);
            assert_eq!(data.param_index, 0);
        } else {
            panic!("Expected PARAM_VALUE message");
        }
    }

    #[test]
    fn test_request_read_by_name() {
        let flash = MockFlash::new();
        let handler = ParamHandler::new(flash);

        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        let request = PARAM_REQUEST_READ_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_index: -1,
        };

        let response = handler.handle_request_read(&request);
        assert!(response.is_some());

        if let Some(MavMessage::PARAM_VALUE(data)) = response {
            assert_eq!(data.param_value, 10.0); // Default value
            assert_eq!(data.param_type, MavParamType::MAV_PARAM_TYPE_UINT32);
        } else {
            panic!("Expected PARAM_VALUE message");
        }
    }

    #[test]
    fn test_request_read_by_index() {
        let flash = MockFlash::new();
        let handler = ParamHandler::new(flash);

        let request = PARAM_REQUEST_READ_DATA {
            target_system: 1,
            target_component: 1,
            param_id: [0; 16],
            param_index: 0,
        };

        let response = handler.handle_request_read(&request);
        assert!(response.is_some());
    }

    #[test]
    fn test_param_set_valid() {
        let flash = MockFlash::new();
        let mut handler = ParamHandler::new(flash);

        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_value: 20.0,
            param_type: MavParamType::MAV_PARAM_TYPE_UINT32,
        };

        let result = handler.handle_set(&set_msg);
        assert!(result.is_ok());

        // Verify value changed
        let param = handler.registry().get_by_name("SR_EXTRA1").unwrap();
        assert_eq!(param.value, ParamValue::Uint32(20));
        assert!(param.modified);
    }

    #[test]
    fn test_param_set_out_of_bounds() {
        let flash = MockFlash::new();
        let mut handler = ParamHandler::new(flash);

        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_value: 100.0, // Max is 50
            param_type: MavParamType::MAV_PARAM_TYPE_UINT32,
        };

        let result = handler.handle_set(&set_msg);
        assert!(matches!(result, Err(ParamHandlerError::InvalidValue)));

        // Verify value unchanged
        let param = handler.registry().get_by_name("SR_EXTRA1").unwrap();
        assert_eq!(param.value, ParamValue::Uint32(10)); // Still default
    }

    #[test]
    fn test_param_set_not_found() {
        let flash = MockFlash::new();
        let mut handler = ParamHandler::new(flash);

        let mut param_id = [0u8; 16];
        param_id[..11].copy_from_slice(b"NONEXISTENT"); // "NONEXISTENT" is 11 bytes

        let set_msg = PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_value: 10.0,
            param_type: MavParamType::MAV_PARAM_TYPE_UINT32,
        };

        let result = handler.handle_set(&set_msg);
        assert!(matches!(result, Err(ParamHandlerError::NotFound)));
    }
}
