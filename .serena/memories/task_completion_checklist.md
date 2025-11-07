# Task Completion Checklist

## When Finishing Rust Code

Run the following commands in order and fix any issues:

1. `cargo fmt` - Auto-format code
2. `cargo clippy --all-targets -- -D warnings` - Check for linting errors
3. `cargo test --lib --quiet` - Run unit tests

All commands must pass successfully before considering the work complete.

## When Finishing Markdown Documentation

Run the following commands in order:

1. `bun scripts/trace-status.ts --check` - Verify traceability integrity
   - Checks for missing/incorrect links between documents
   - Validates dependency consistency
   - Ensures task reciprocal links are correct
   - Fix any issues before proceeding

2. `bun format` - Auto-format markdown files
   - Automatically fixes formatting issues
   - Ensures consistent markdown style

3. `bun lint` - Check markdown linting
   - Identifies potential issues and violations
   - Fix any warnings or errors reported

All commands must pass successfully. After finalization, compare against the source template to confirm metadata, links, and status selections remain consistent.

## Additional Checks

### If docs/traceability.md is Missing or Files Added/Removed
```bash
bun scripts/trace-status.ts --write
```

### During Implementation
- Mark completed phase checkboxes in relevant documents at the end of each phase
- Keep progress transparent and auditable

## TDL Workflow Approvals

Every stage requires explicit approval before proceeding:
1. Analysis → Requirements
2. Requirements → ADR
3. ADR → Task Package
4. Task Package → Implementation
5. Each implementation phase → Next phase

Stop and request approval at each checkpoint. Do not proceed without explicit permission.
