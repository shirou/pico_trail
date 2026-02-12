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
use crate::traits::flash::FlashInterface;
use mavlink::common::{
    MavMessage, MavParamType, PARAM_REQUEST_LIST_DATA, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA,
    PARAM_VALUE_DATA,
};

/// Parameter handler error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// Create a parameter handler from an existing store
    ///
    /// Use this when parameter registration is handled externally
    /// (e.g., firmware-specific parameter modules).
    pub fn from_store(store: ParameterStore) -> Self {
        Self { store }
    }

    /// Get parameter count (excluding hidden parameters)
    pub fn count(&self) -> usize {
        self.store.count()
    }

    /// Handle PARAM_REQUEST_LIST message
    ///
    /// Returns a vector of PARAM_VALUE messages for all non-hidden parameters.
    pub fn handle_request_list(
        &self,
        _data: &PARAM_REQUEST_LIST_DATA,
    ) -> heapless::Vec<MavMessage, 64> {
        let mut temp_params = heapless::Vec::<(&str, &ParamValue), 64>::new();

        for name in self.store.iter_names() {
            if let Some(value) = self.store.get(name.as_str()) {
                match value {
                    ParamValue::String(_) => continue,
                    _ => {
                        let _ = temp_params.push((name.as_str(), value));
                    }
                }
            }
        }

        let count = temp_params.len() as u16;

        let mut messages = heapless::Vec::new();
        for (index, (name, value)) in temp_params.iter().enumerate() {
            if let Some(msg) = self.create_param_value_message(name, value, index as u16, count) {
                let _ = messages.push(msg);
            }
        }

        messages
    }

    /// Handle PARAM_REQUEST_READ message
    ///
    /// Returns a PARAM_VALUE message for the requested parameter.
    /// Hidden parameters return None (not readable).
    pub fn handle_request_read(&self, data: &PARAM_REQUEST_READ_DATA) -> Option<MavMessage> {
        let mut sendable_params = heapless::Vec::<heapless::String<16>, 64>::new();
        for name in self.store.iter_names() {
            if let Some(value) = self.store.get(name.as_str()) {
                if !matches!(value, ParamValue::String(_)) {
                    let _ = sendable_params.push(name.clone());
                }
            }
        }
        let total_count = sendable_params.len() as u16;

        if data.param_index >= 0 {
            let index = data.param_index as usize;
            if index < sendable_params.len() {
                let name = &sendable_params[index];
                if let Some(value) = self.store.get(name.as_str()) {
                    return self.create_param_value_message(
                        name.as_str(),
                        value,
                        index as u16,
                        total_count,
                    );
                }
            }
            return None;
        }

        let param_id = core::str::from_utf8(&*data.param_id)
            .ok()?
            .trim_end_matches('\0');

        if self.store.is_hidden(param_id) {
            return None;
        }

        if let Some(value) = self.store.get(param_id) {
            if matches!(value, ParamValue::String(_)) {
                return None;
            }

            for (index, name) in sendable_params.iter().enumerate() {
                if name.as_str() == param_id {
                    return self.create_param_value_message(
                        param_id,
                        value,
                        index as u16,
                        total_count,
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
    pub fn handle_set(&mut self, data: &PARAM_SET_DATA) -> Result<MavMessage, ParamHandlerError> {
        let param_id = core::str::from_utf8(&*data.param_id)
            .map_err(|_| ParamHandlerError::NotFound)?
            .trim_end_matches('\0');

        let value = match data.param_type {
            MavParamType::MAV_PARAM_TYPE_REAL32 => ParamValue::Float(data.param_value),
            MavParamType::MAV_PARAM_TYPE_INT32 => {
                ParamValue::Int(data.param_value.to_bits() as i32)
            }
            MavParamType::MAV_PARAM_TYPE_UINT32 => {
                let value_u32 = data.param_value.to_bits();

                if param_id == "NET_IP" || param_id == "NET_NETMASK" || param_id == "NET_GATEWAY" {
                    ParamValue::Ipv4(value_u32.to_be_bytes())
                } else {
                    ParamValue::Int(value_u32 as i32)
                }
            }
            MavParamType::MAV_PARAM_TYPE_UINT8 => {
                ParamValue::Int((data.param_value.to_bits() & 0xFF) as i32)
            }
            _ => return Err(ParamHandlerError::InvalidValue),
        };

        self.store
            .set(param_id, value)
            .map_err(|_| ParamHandlerError::StoreError)?;

        let updated_value = self
            .store
            .get(param_id)
            .ok_or(ParamHandlerError::NotFound)?;

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
        let mut param_id = [0u8; 16];
        let name_bytes = name.as_bytes();
        let copy_len = name_bytes.len().min(16);
        param_id[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        let (param_value, param_type) = match value {
            ParamValue::Float(f) => (*f, MavParamType::MAV_PARAM_TYPE_REAL32),
            ParamValue::Int(i) => (
                f32::from_bits(*i as u32),
                MavParamType::MAV_PARAM_TYPE_INT32,
            ),
            ParamValue::Bool(b) => {
                let value_u8 = if *b { 1u32 } else { 0u32 };
                (f32::from_bits(value_u8), MavParamType::MAV_PARAM_TYPE_UINT8)
            }
            ParamValue::Ipv4(ip) => {
                let value_u32 = u32::from_be_bytes(*ip);
                (
                    f32::from_bits(value_u32),
                    MavParamType::MAV_PARAM_TYPE_UINT32,
                )
            }
            ParamValue::String(_) => {
                return None;
            }
        };

        Some(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value,
            param_count: count,
            param_index: index,
            param_id: param_id.into(),
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
    pub fn save_to_flash<F: FlashInterface>(
        &mut self,
        flash: &mut F,
    ) -> Result<(), ParamHandlerError> {
        crate::parameters::storage::save_to_flash(&mut self.store, flash)
            .map_err(|_| ParamHandlerError::StoreError)
    }
}
