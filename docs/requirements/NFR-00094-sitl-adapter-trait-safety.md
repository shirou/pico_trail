# NFR-00094 SITL Adapter Trait Safety

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Category: Extensibility

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](FR-00148-sitl-simulator-adapter-trait.md)
- Related Tasks:
  - [T-00157-sitl-simulator-integration](../tasks/T-00157-sitl-simulator-integration/README.md)

## Requirement Statement

The `SimulatorAdapter` trait shall be object-safe and thread-safe (`Send + Sync`), enabling dynamic adapter registration and safe concurrent access across async tasks.

## Rationale

Dynamic dispatch (`Box<dyn SimulatorAdapter>`) is required for runtime adapter selection. Thread safety is essential because:

1. Adapter registry may be accessed from multiple async tasks
2. Lockstep coordinator coordinates multiple adapters concurrently
3. Future multi-threaded execution models require safe sharing

## Category-Specific Details

### Extensibility Requirements

- `Box<dyn SimulatorAdapter>` must compile without errors
- Trait can be implemented by external crates
- No `Self: Sized` restrictions on trait methods
- Associated types (if any) must be object-safe

## Acceptance Criteria

- [ ] `SimulatorAdapter: Send + Sync` compiles successfully
- [ ] `Box<dyn SimulatorAdapter>` usable as HashMap value type
- [ ] Adapter can be sent between async tasks
- [ ] Adapter can be shared via `Arc<Mutex<dyn SimulatorAdapter>>`
- [ ] No `Self: Sized` bounds on any trait method
- [ ] Compile-time verification via unit tests

## Technical Details (if applicable)

### Trait Definition Constraints

```rust
// This must compile
#[async_trait]
pub trait SimulatorAdapter: Send + Sync {
    // All methods must be object-safe
    fn adapter_type(&self) -> &'static str;
    fn name(&self) -> &str;

    async fn connect(&mut self) -> Result<(), SimulatorError>;
    // ...
}

// This usage pattern must work
fn register(adapters: &mut HashMap<String, Box<dyn SimulatorAdapter>>) {
    adapters.insert("test".to_string(), Box::new(TestAdapter::new()));
}

// Cross-task sending must work
async fn spawn_task(adapter: Box<dyn SimulatorAdapter>) {
    tokio::spawn(async move {
        // Use adapter
    });
}
```

### Object Safety Verification

```rust
#[test]
fn trait_is_object_safe() {
    fn assert_object_safe<T: ?Sized>() {}
    assert_object_safe::<dyn SimulatorAdapter>();
}

#[test]
fn trait_is_send_sync() {
    fn assert_send<T: Send>() {}
    fn assert_sync<T: Sync>() {}
    assert_send::<Box<dyn SimulatorAdapter>>();
    assert_sync::<Box<dyn SimulatorAdapter>>();
}
```

## Measurement / Validation

| Metric        | Target   | Measurement Method                  |
| ------------- | -------- | ----------------------------------- |
| Object safety | Compiles | `Box<dyn SimulatorAdapter>` in code |
| Send bound    | Compiles | `tokio::spawn` with adapter         |
| Sync bound    | Compiles | `Arc<dyn SimulatorAdapter>` in code |

## Risks & Mitigation

| Risk                                 | Impact | Likelihood | Mitigation                                 |
| ------------------------------------ | ------ | ---------- | ------------------------------------------ |
| async_trait breaks object safety     | High   | Low        | Use `#[async_trait]` macro consistently    |
| Future method addition breaks safety | Medium | Low        | Review all trait changes for object safety |

## Implementation Notes

- `async_trait` crate handles the complexity of async in object-safe traits
- Avoid generic methods (they break object safety)
- Avoid associated types without bounds

## External References

- [Rust Object Safety](https://doc.rust-lang.org/reference/items/traits.html#object-safety)
- [async_trait crate](https://docs.rs/async-trait/)
