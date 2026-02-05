# FR-00149 SITL Multi-Adapter Registration

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](FR-00148-sitl-simulator-adapter-trait.md)
- Dependent Requirements:
  - [FR-00150-sitl-multi-vehicle-instances](FR-00150-sitl-multi-vehicle-instances.md)
  - [FR-00153-sitl-lockstep-synchronization](FR-00153-sitl-lockstep-synchronization.md)
  - [NFR-00097-sitl-adapter-extensibility](NFR-00097-sitl-adapter-extensibility.md)
- Related Tasks:
  - [T-00157-sitl-simulator-integration](../tasks/T-00157-sitl-simulator-integration/README.md)

## Requirement Statement

SITL Bridge shall support registering multiple simulator adapters simultaneously. Each adapter shall be identified by a unique name, and vehicles can be assigned to specific adapters at runtime.

## Rationale

Supporting multiple adapters enables:

1. Running different simulators for different vehicles in the same session
2. Comparing simulator behaviors side-by-side
3. Using lightweight adapter for some tests while Gazebo runs for others
4. Gradual migration between simulator versions

## User Story (if applicable)

As a developer testing multi-vehicle scenarios, I want to run some vehicles in Gazebo for visualization while others use the lightweight adapter for faster iteration.

## Acceptance Criteria

- [ ] `SitlBridge` maintains a registry of adapters by name
- [ ] `register_adapter()` adds an adapter to the registry
- [ ] `unregister_adapter()` removes an adapter from the registry
- [ ] `get_adapter()` retrieves an adapter by name
- [ ] `list_adapters()` returns all registered adapter names
- [ ] Adapter names must be unique (error on duplicate registration)
- [ ] Vehicles can be assigned to adapters via `assign_vehicle_to_adapter()`
- [ ] Multiple vehicles can share the same adapter
- [ ] Adapters can be registered/unregistered while simulation is paused
- [ ] Unit tests for registry operations

## Technical Details (if applicable)

### Bridge Registry Interface

```rust
impl SitlBridge {
    pub fn register_adapter(&mut self, adapter: Box<dyn SimulatorAdapter>) -> Result<(), SitlError>;
    pub fn unregister_adapter(&mut self, name: &str) -> Result<(), SitlError>;
    pub fn get_adapter(&self, name: &str) -> Option<&dyn SimulatorAdapter>;
    pub fn get_adapter_mut(&mut self, name: &str) -> Option<&mut dyn SimulatorAdapter>;
    pub fn list_adapters(&self) -> Vec<&str>;
    pub fn assign_vehicle_to_adapter(&mut self, vehicle_id: VehicleId, adapter_name: &str) -> Result<(), SitlError>;
}
```

### Internal Storage

```rust
pub struct SitlBridge {
    adapters: HashMap<String, Box<dyn SimulatorAdapter>>,
    vehicle_adapter_map: HashMap<VehicleId, String>,
    // ...
}
```

## Platform Considerations

### Cross-Platform

- Registry logic is pure Rust with no platform dependencies
- HashMap for O(1) adapter lookup by name

## Risks & Mitigation

| Risk                         | Impact | Likelihood | Mitigation                            | Validation |
| ---------------------------- | ------ | ---------- | ------------------------------------- | ---------- |
| Adapter name collisions      | Low    | Low        | Return error on duplicate             | Unit tests |
| Dangling vehicle assignments | Medium | Low        | Validate adapter exists on assignment | Unit tests |

## Implementation Notes

- Adapters are stored as `Box<dyn SimulatorAdapter>` for dynamic dispatch
- Adapter type is retrieved via `adapter_type()` method, name via `name()`
- Consider adding adapter lifecycle hooks (on_register, on_unregister)

## External References

- None
