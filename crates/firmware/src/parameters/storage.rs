//! Flash-backed Parameter Storage
//!
//! Re-exports core parameter types and flash persistence functions.
//! Core types (`ParameterStore`, `ParamValue`, `ParamFlags`, `ParamMetadata`)
//! and flash persistence (`load_from_flash`, `save_to_flash`) are defined in
//! `pico_trail_core::parameters::storage`.

pub use pico_trail_core::parameters::storage::{
    load_from_flash, save_to_flash, ParamFlags, ParamValue, ParameterStore, MAX_PARAMS,
    MAX_STRING_LEN, PARAM_BLOCK_BASE, PARAM_BLOCK_COUNT, PARAM_BLOCK_SIZE, PARAM_MAGIC,
    PARAM_NAME_LEN, PARAM_VERSION,
};
pub use pico_trail_core::parameters::ParamMetadata;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::flash::MockFlash;

    #[test]
    fn test_param_value_serialization() {
        // Serialization is tested via round-trip through flash
        let mut store = ParameterStore::new();
        store
            .register("TEST_INT", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();

        // Mark dirty so save works (change a value)
        store.set("TEST_INT", ParamValue::Int(99)).unwrap();
        assert!(store.is_dirty());

        let mut flash = MockFlash::new();
        save_to_flash(&mut store, &mut flash).unwrap();
        assert!(!store.is_dirty()); // Should be cleared after save

        let loaded = load_from_flash(&mut flash).unwrap();
        assert_eq!(loaded.get("TEST_INT"), Some(&ParamValue::Int(99)));
    }

    #[test]
    fn test_parameter_store_basic() {
        let mut store = ParameterStore::new();

        // Register parameter
        store
            .register("TEST", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();

        // Get parameter
        assert_eq!(store.get("TEST"), Some(&ParamValue::Int(42)));

        // Set parameter
        store.set("TEST", ParamValue::Int(100)).unwrap();
        assert_eq!(store.get("TEST"), Some(&ParamValue::Int(100)));
        assert!(store.is_dirty());
    }

    #[test]
    fn test_parameter_hidden() {
        let mut store = ParameterStore::new();

        // Register hidden parameter
        store
            .register(
                "SECRET",
                ParamValue::String(heapless::String::try_from("password").unwrap()),
                ParamFlags::HIDDEN,
            )
            .unwrap();

        // Check if hidden
        assert!(store.is_hidden("SECRET"));
        assert_eq!(store.count(), 0); // Hidden parameters not counted
    }

    #[test]
    fn test_parameter_read_only() {
        let mut store = ParameterStore::new();

        // Register read-only parameter
        store
            .register("READONLY", ParamValue::Int(42), ParamFlags::READ_ONLY)
            .unwrap();

        // Try to set (should fail)
        assert!(store.set("READONLY", ParamValue::Int(100)).is_err());
    }
}
