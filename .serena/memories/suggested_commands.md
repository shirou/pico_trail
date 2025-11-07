# Suggested Commands

## Rust Development

### Code Quality (ALWAYS run when finishing Rust code)
```bash
cargo fmt                                    # Auto-format code
cargo clippy --all-targets -- -D warnings    # Lint with strict warnings
cargo test --lib --quiet                     # Run unit tests (fast)
```

### Building
```bash
cargo build                           # Debug build (host)
cargo build --release                 # Release build (host)
```

### Building for Embedded (RP2350/Pico 2 W)
```bash
# Build all examples
./scripts/build-rp2350.sh

# Build specific example
./scripts/build-rp2350.sh scheduler_demo

# Release build
./scripts/build-rp2350.sh --release scheduler_demo
```

### Flashing to Pico 2 W
```bash
# Method 1: Using probe-rs (recommended - shows defmt logs)
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/scheduler_demo

# Method 2: Manual UF2 copy
# 1. Hold BOOTSEL button while connecting USB
# 2. Copy target/scheduler_demo.uf2 to mounted drive
# 3. Pico will automatically reboot
```

### Testing
```bash
cargo test --lib --quiet              # Unit tests only (fast)
cargo test                            # All tests
```

## Documentation

### Markdown Quality (ALWAYS run when finishing docs)
```bash
bun scripts/trace-status.ts --check   # Verify traceability
bun format                            # Auto-format markdown
bun lint                              # Check markdown linting
```

### Traceability
```bash
# Generate/update traceability.md
bun scripts/trace-status.ts --write
```

## System Commands

Standard Linux commands available:
- `git` - Version control
- `ls`, `cd`, `pwd` - Directory navigation
- `grep`, `find` - File searching
- `cat`, `head`, `tail` - File viewing
- `date` - Current date/time
