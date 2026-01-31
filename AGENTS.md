# AGENTS.md

## Documentation Language Policy

All documentation output in this project must be written in English, including:

- Code comments
- Commit messages
- Architecture Decision Records (ADRs)
- README files
- API documentation
- Error messages
- User-facing documentation
- Test descriptions
- TODO comments
- Any other written documentation

## Project Overview

### User Documentation

## Agent Operating Environment

### Planning and Tooling Discipline

## Developer Principles

### Memory Safety Over Micro-optimization

- Prioritize memory safety and correctness over micro-optimizations
- Accept reasonable overhead (e.g., cloning small strings) to avoid memory leaks
- Follow Rust's ownership model strictly - avoid `unsafe` code and memory leaks from techniques like `Box::leak()`
- When faced with lifetime complexity, prefer simpler solutions that may use slightly more memory but are correct
- Example: Clone strings for HTTP headers instead of using `Box::leak()` to create static references

### Code Clarity

- Write clear, readable code that is easy to understand and maintain
- Use descriptive variable and function names
- Add comments for complex logic, but prefer self-documenting code
- Structure code to minimize cognitive load for future developers

### Clean Code Maintenance

- Remove unused variables, parameters, and struct members promptly
- When refactoring, trace through all callers to eliminate unnecessary parameters
- Keep structs lean by removing fields that are no longer used
- Use `cargo clippy` to identify unused code elements
- Example: If a function parameter like `arch` is no longer used in the implementation, remove it from the function signature and update all callers

### Prefer Functions Over Structs Without State

- When there's no state to manage, prefer implementing functionality as standalone functions rather than defining structs
- Only create structs when you need to maintain state, implement traits, or group related data
- This keeps the code simpler and more straightforward
- Example: For utility operations like file validation or string parsing, use functions directly instead of creating a struct with methods

### Avoid Generic "Manager" Naming

- When the name "manager" appears in file names, structs, traits, or similar constructs, consider more specific and descriptive alternatives
- "Manager" is often too abstract and doesn't clearly communicate the responsibility
- Choose names that describe what the component actually does
- Examples of better alternatives:
  - `FileManager` → `FileSystem`, `FileStore`, `FileRepository`
  - `ConnectionManager` → `ConnectionPool`, `ConnectionFactory`
  - `TaskManager` → `TaskScheduler`, `TaskExecutor`, `TaskQueue`
  - `ShimManager` → `ShimInstaller`, `ShimRegistry`, `ShimProvisioner`
- This principle helps maintain code clarity and makes the codebase more intuitive

### Avoid Vague "Util" or "Utils" Naming

- Never use "util" or "utils" in directory names, file names, class names, or variable names
- These terms are too generic and don't clearly convey the purpose or responsibility
- Always choose specific names that describe the actual functionality
- Examples of better alternatives:
  - `utils/strings.rs` → `string_operations.rs`, `text_processing.rs`, `string_formatter.rs`
  - `FileUtils` → `FileOperations`, `FileSystem`, `PathValidator`
  - `DateUtil` → `DateFormatter`, `DateParser`, `TimeCalculator`
  - `CommonUtils` → Split into specific modules based on functionality
  - `util_function()` → Name based on what it does: `validate_input()`, `format_output()`
- This principle ensures code is self-documenting and responsibilities are clear

### Module Placement Consistency

- Consult `docs/architecture.md` before creating or moving modules so directory structure stays aligned with the documented layout.
- Keep platform-dependent code under `src/platform/` (and its submodules) and expose only cross-platform interfaces from higher layers.
- When introducing new components, document their location rationale in the relevant design or plan to aid future maintainers.

### Prevent Circular Module Dependencies

- Keep the module graph acyclic so features remain testable and maintainable.
- Favor dependency inversion (traits, interfaces) or data transfer structures instead of bidirectional imports when modules must collaborate.
- If a new dependency would close a cycle, refactor by extracting shared functionality into a dedicated module documented in the architecture references.
- Run dependency analysis tools or targeted `cargo check` commands when restructuring to confirm cycles are not introduced.

### Logging Standards

- **Always use the abstracted logging macros** defined in `src/core/logging.rs` for all log output
- **Never use `defmt` directly** - use the crate-level macros instead:
  - `crate::log_info!(...)` - Informational messages
  - `crate::log_warn!(...)` - Warning messages
  - `crate::log_error!(...)` - Error messages
  - `crate::log_debug!(...)` - Debug messages
  - `crate::log_trace!(...)` - Trace-level messages
- **DO NOT add `#[cfg(feature = "pico2_w")]`** guards around logging calls - the macros handle target-specific behavior internally
- The logging macros automatically:
  - Use `defmt` on embedded targets (pico2_w feature)
  - Use `println!` in host tests
  - Compile to no-op in other contexts
- This abstraction centralizes conditional compilation and keeps logging code clean and portable

### MAVLink Protocol Handling

- **All MAVLink message handling MUST be implemented in `src/communication/mavlink/`**
- **Never process MAVLink messages directly in examples or application code**
- Follow the established handler pattern:
  - Create or extend handlers in `src/communication/mavlink/handlers/`
  - Register handlers with `MessageDispatcher` in `src/communication/mavlink/dispatcher.rs`
  - Use `dispatcher.process_rc_input()` for async RC messages (RC_CHANNELS, RC_CHANNELS_OVERRIDE)
  - Use `dispatcher.dispatch()` for synchronous messages that produce responses
- **Examples should only:**
  - Initialize handlers and dispatcher
  - Call dispatcher methods to process messages
  - Send responses returned by the dispatcher
  - NOT extract message data or implement protocol logic
- This centralization ensures:
  - Protocol logic is reusable across all examples and applications
  - Testing is easier with handlers in one location
  - Changes to MAVLink handling don't require updating multiple files
  - Examples remain simple and focused on demonstrating usage
- **Example anti-pattern**: Extracting channel data from RC_CHANNELS in the example file
- **Correct pattern**: Call `dispatcher.process_rc_input(&msg, timestamp)` and let the handler extract data

## Traceable Development Lifecycle (TDL)

This project follows the Traceable Development Lifecycle (TDL). For TDL-related tasks (analysis, requirements, ADRs, task packages, implementation phases), use the `/tdl` skill which provides the full workflow, templates, and approval gates.

## Development Workflow

### Completing Work

#### Rust Code

When finishing any Rust coding task, always run the following commands in order and fix any issues:

1. `cargo fmt` - Auto-format code
2. `cargo clippy --all-targets -- -D warnings` - Check for linting errors in test code
3. `cargo test --lib --quiet` - Run unit tests (faster than full test suite)
4. `./scripts/build-rp2350.sh pico_trail_rover` - Verify embedded build works (RP2350/Pico 2 W)

Address any errors from each command before proceeding to the next. All must pass successfully before considering the work complete.

**Note**: The RP2350 build (step 4) is critical for catching embedded-specific issues (e.g., feature gates, platform-specific imports). Host tests alone are not sufficient.

#### Markdown Documentation

When working on Markdown documentation (`.md` files), run the following commands in order:

1. `bun scripts/trace-status.ts --check` - Verify traceability integrity
   - Checks for missing or incorrect links between documents
   - Validates dependency consistency (prerequisite/dependent requirements)
   - Ensures task reciprocal links are correct
   - Fix any issues reported before proceeding to formatting

2. `bun format` - Auto-format markdown files
   - Automatically fixes formatting issues
   - Ensures consistent markdown style across all documentation

3. `bun lint` - Check markdown linting
   - Identifies potential issues and violations
   - Common issues: trailing spaces, inconsistent indentation, missing blank lines
   - Fix any warnings or errors reported

All commands must pass successfully before considering the documentation work complete. After the document is finalized, compare it against the source template (for example, `docs/templates/analysis.md`) to confirm the Metadata, Links, and status selections remain consistent with the current standards.

### Documentation Updates

- Ensure documentation, comments, and messages remain in English.
- For Markdown changes, run `bun scripts/trace-status.ts --check` to verify traceability, then `bun format` followed by `bun lint` and resolve any reported issues before finalizing.
- If `docs/traceability.md` is missing or you add or remove any files under `docs/`, run `bun scripts/trace-status.ts --write` to regenerate the traceability matrix before completing the work.
- During implementation, mark the completed phase checkboxes in the relevant documents at the end of each phase so progress stays transparent and auditable.

## Essential Commands

- **Format**: `cargo fmt` - Format code using rustfmt
- **Lint**: `cargo clippy --all-targets -- -D warnings` - Run linter with strict warnings
- **Build**: `cargo build` (debug), `cargo build --release` (production)
- **Test**: `cargo test --lib --quiet` - Run unit tests efficiently

### Building for Embedded Targets (RP2350)

**IMPORTANT: Always use the build script for RP2350/Pico 2 W targets.**

- **Build examples**: `./scripts/build-rp2350.sh` - Build all examples and convert to UF2
- **Build specific example**: `./scripts/build-rp2350.sh scheduler_demo` - Build single example
- **Release build**: `./scripts/build-rp2350.sh --release scheduler_demo` - Optimized build

The script automatically:

1. Builds the example for `thumbv8m.main-none-eabihf` target
2. Converts ELF binary to UF2 format (required for flashing)
3. Places UF2 files in `target/` directory

**Flashing to Pico 2 W:**

```bash
# Method 1: Using probe-rs (recommended - shows defmt logs)
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/scheduler_demo

# Method 2: Manual UF2 copy
# 1. Hold BOOTSEL button while connecting USB
# 2. Copy target/scheduler_demo.uf2 to mounted drive
# 3. Pico will automatically reboot and run
```

**Note**: Host tests and embedded builds use different targets. Do not use the default `cargo build` for embedded code.

## Additional Documentation

- **Architecture & Structure**: [`docs/architecture.md`](docs/architecture.md) - Project structure, components, and storage locations

## Communication Guidelines

- Default to concise, friendly teammate tone; structure responses for quick scanning without over-formatting.
- Lead code-change summaries with the key outcome, then reference affected paths with `path:line` format (no ranges).
- Use bullets (`-`) for lists, avoid nested bullets, and reserve headings for major sections only when helpful.
- Include suggested next steps (tests, commits, verification) when they naturally follow from the work performed.
- Do not paste entire file contents; reference file paths instead.
- When the user requests command output, summarize relevant lines rather than dumping full logs.
- Execute simple user requests via shell commands when appropriate (e.g., `date`), respecting the environment rules above.
