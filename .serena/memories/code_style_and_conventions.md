# Code Style and Conventions

## Core Principles

### Memory Safety Over Micro-optimization
- Prioritize memory safety and correctness over micro-optimizations
- Accept reasonable overhead (e.g., cloning small strings) to avoid memory leaks
- Follow Rust's ownership model strictly - avoid `unsafe` code
- Avoid techniques like `Box::leak()` for creating static references

### Code Clarity
- Write clear, readable code that is easy to understand and maintain
- Use descriptive variable and function names
- Add comments for complex logic, but prefer self-documenting code
- Structure code to minimize cognitive load

### Clean Code Maintenance
- Remove unused variables, parameters, and struct members promptly
- When refactoring, trace through all callers to eliminate unnecessary parameters
- Keep structs lean by removing fields that are no longer used
- Use `cargo clippy` to identify unused code elements

### Prefer Functions Over Structs Without State
- When there's no state to manage, prefer standalone functions over structs
- Only create structs when you need to maintain state, implement traits, or group related data
- Example: For utility operations like file validation or string parsing, use functions directly

### Avoid Generic Naming
- **Never use "Manager"**: Use specific names (e.g., `ConnectionPool` instead of `ConnectionManager`)
- **Never use "Util" or "Utils"**: Use specific names (e.g., `StringFormatter` instead of `StringUtil`)
- Choose names that describe what the component actually does

## Rust-Specific

- Follow standard Rust naming conventions (snake_case, CamelCase)
- Use `rustfmt` for automatic formatting
- Address all `clippy` warnings (strict mode: `-D warnings`)
- Write unit tests for new functionality
- Use traits for abstraction and polymorphism

## Documentation

- All documentation must be in English (code comments, commit messages, ADRs, etc.)
- Follow TDL process for structured documentation
- Keep traceability between documents
