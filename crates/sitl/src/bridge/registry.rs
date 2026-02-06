use std::collections::HashMap;

use crate::adapter::SimulatorAdapter;
use crate::error::SimulatorError;

/// Registry for simulator adapter instances.
pub struct AdapterRegistry {
    adapters: HashMap<String, Box<dyn SimulatorAdapter>>,
}

impl AdapterRegistry {
    pub fn new() -> Self {
        Self {
            adapters: HashMap::new(),
        }
    }

    /// Register a new adapter. Returns error if an adapter with the same name exists.
    pub fn register(&mut self, adapter: Box<dyn SimulatorAdapter>) -> Result<(), SimulatorError> {
        let name = adapter.name().to_string();
        if self.adapters.contains_key(&name) {
            return Err(SimulatorError::AdapterAlreadyRegistered(name));
        }
        self.adapters.insert(name, adapter);
        Ok(())
    }

    /// Unregister an adapter by name. Returns error if not found.
    pub fn unregister(&mut self, name: &str) -> Result<(), SimulatorError> {
        if self.adapters.remove(name).is_none() {
            return Err(SimulatorError::AdapterNotFound(name.to_string()));
        }
        Ok(())
    }

    /// List all registered adapter names.
    pub fn list(&self) -> Vec<&str> {
        self.adapters.keys().map(|s| s.as_str()).collect()
    }

    /// Get a reference to an adapter by name.
    pub fn get(&self, name: &str) -> Option<&dyn SimulatorAdapter> {
        self.adapters.get(name).map(|a| a.as_ref())
    }

    /// Get a mutable reference to an adapter by name.
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Box<dyn SimulatorAdapter>> {
        self.adapters.get_mut(name)
    }
}

impl Default for AdapterRegistry {
    fn default() -> Self {
        Self::new()
    }
}
