# Codebase Structure

## Source Directory Layout

```
src/
├── platform/          # Hardware abstraction layer (UART, I2C, SPI, PWM)
├── devices/           # Device drivers (GPS, IMU, Motor, Servo)
├── subsystems/        # Functional subsystems (AHRS, Control, Navigation)
├── vehicle/           # Vehicle logic and flight modes
└── core/              # Cross-cutting concerns (Scheduler, Parameters, Logger)
```

## Architecture Layers (Top to Bottom)

1. **Application Layer**: Rover/Boat modes & mission management
2. **Communication Layer**: MAVLink protocol, telemetry, parameters
3. **Control & Navigation Layer**: AHRS, PID, waypoint navigation, path following
4. **Device Driver Layer**: GPS, IMU, Motor, Servo device abstractions
5. **Hardware Abstraction Layer (Platform HAL)**: UART, I2C, SPI, PWM (platform-specific)

## Key Directories

- **docs/**: TDL documentation (analysis, requirements, ADRs, tasks)
  - `docs/analysis/` - Problem exploration (AN-*.md)
  - `docs/requirements/` - FR-*.md, NFR-*.md
  - `docs/adr/` - Architecture Decision Records (ADR-*.md)
  - `docs/tasks/` - Task packages (T-*/design.md, plan.md)
  - `docs/templates/` - Document templates
  
- **examples/**: Example applications (mavlink_demo, scheduler_demo, etc.)
- **scripts/**: Build and utility scripts
  - `build-rp2350.sh` - Build examples for RP2350
  - `trace-status.ts` - Verify traceability
  
- **tests/**: Integration tests
- **config/**: Configuration files
- **.cargo/**: Cargo configuration (target, runner)

## Module Placement Rules

- Platform-dependent code goes in `src/platform/` (and submodules)
- Expose only cross-platform interfaces from higher layers
- Keep module graph acyclic (no circular dependencies)
- Consult `docs/architecture.md` before creating/moving modules
