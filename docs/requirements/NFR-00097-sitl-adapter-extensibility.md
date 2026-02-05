# NFR-00097 SITL Adapter Extensibility

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Category: Extensibility

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](FR-00148-sitl-simulator-adapter-trait.md)
  - [FR-00149-sitl-multi-adapter-registration](FR-00149-sitl-multi-adapter-registration.md)
- Related Tasks:
  - [T-00157-sitl-simulator-integration](../tasks/T-00157-sitl-simulator-integration/README.md)

## Requirement Statement

Adding a new simulator adapter shall not require modifying SITL Bridge core code. A new adapter is implemented as a struct implementing `SimulatorAdapter` trait and registered at runtime.

## Rationale

Extensibility enables:

1. Community contributions without core codebase changes
2. Custom/proprietary simulator integrations
3. Experimentation with new simulators
4. Plugin-style architecture

## Category-Specific Details

### Extensibility Requirements

- New adapter = new file implementing trait
- No changes to `SitlBridge` source code required
- No changes to existing adapters required
- Registration via public API only

## Acceptance Criteria

- [ ] New adapter implemented in separate module/crate
- [ ] No modifications to `crates/sitl/src/bridge.rs` required
- [ ] No modifications to existing adapters required
- [ ] Adapter registered via `bridge.register_adapter(Box::new(MyAdapter::new()))`
- [ ] Adapter discoverable via `bridge.list_adapters()`
- [ ] Documentation explains how to add new adapters
- [ ] Example adapter provided as template

## Technical Details (if applicable)

### Adding a New Adapter (User Guide)

```rust
// my_adapter.rs - No changes to pico_trail codebase needed

use pico_trail_sitl::{SimulatorAdapter, SensorData, ActuatorCommands, SimulatorError};

pub struct MyCustomAdapter {
    // Custom state
}

impl MyCustomAdapter {
    pub fn new(config: MyConfig) -> Self {
        Self { /* ... */ }
    }
}

#[async_trait]
impl SimulatorAdapter for MyCustomAdapter {
    fn adapter_type(&self) -> &'static str { "my-custom" }
    fn name(&self) -> &str { "My Custom Simulator" }

    async fn connect(&mut self) -> Result<(), SimulatorError> {
        // Connect to your simulator
        Ok(())
    }

    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError> {
        // Receive from your simulator, convert to SensorData
        Ok(None)
    }

    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError> {
        // Convert ActuatorCommands to your simulator's format, send
        Ok(())
    }

    // ... implement remaining methods
}
```

### Registration (User Code)

```rust
// main.rs or test setup
use pico_trail_sitl::SitlBridge;
use my_adapter::MyCustomAdapter;

fn main() {
    let mut bridge = SitlBridge::new();

    // Register custom adapter - no pico_trail code changes needed
    bridge.register_adapter(Box::new(MyCustomAdapter::new(config)));

    // Use it
    bridge.spawn_vehicle(VehicleConfig {
        // ...
    });
    bridge.assign_vehicle_to_adapter(vehicle_id, "my-custom");
}
```

### Verification Test

```rust
#[test]
fn test_external_adapter_registration() {
    // This test proves no core changes are needed
    struct ExternalAdapter;

    #[async_trait]
    impl SimulatorAdapter for ExternalAdapter {
        // Minimal implementation
    }

    let mut bridge = SitlBridge::new();
    bridge.register_adapter(Box::new(ExternalAdapter)).unwrap();

    assert!(bridge.list_adapters().contains(&"external"));
}
```

## Measurement / Validation

| Metric                       | Target                 | Measurement Method |
| ---------------------------- | ---------------------- | ------------------ |
| Core changes for new adapter | 0 lines                | Git diff review    |
| Registration API calls       | 1 (`register_adapter`) | API review         |
| Documentation completeness   | Full guide             | Doc review         |

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                                |
| ------------------------------------- | ------ | ---------- | ----------------------------------------- |
| Trait changes break external adapters | High   | Low        | Semantic versioning, deprecation warnings |
| Poor adapter quality                  | Low    | Medium     | Provide example adapter, documentation    |

## Implementation Notes

- Consider providing adapter templates/scaffolding
- Version the `SimulatorAdapter` trait carefully
- Document required vs optional trait methods clearly

## External References

- [Rust Plugin Patterns](https://rust-unofficial.github.io/patterns/patterns/behavioural/strategy.html)
