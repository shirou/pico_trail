# Traceable Development Lifecycle (TDL)

## Overview

This project follows TDL, a structured development process with full traceability from discovery through delivery.

## Process Flow

```
Analysis → Requirements → ADR → Task (Design + Plan) → Implementation
```

## Document Locations

- **Templates**: `docs/templates/` - All document templates
- **Analysis**: `docs/analysis/AN-<id>-<topic>.md`
- **Requirements**: 
  - Functional: `docs/requirements/FR-<id>-<capability>.md`
  - Non-functional: `docs/requirements/NFR-<id>-<quality>.md`
- **ADRs**: `docs/adr/ADR-<id>-<title>.md`
- **Tasks**: `docs/tasks/T-<id>-<task>/` (design.md, plan.md, README.md)
- **Traceability**: `docs/traceability.md`

## Approval Gates

**Critical Rule**: Complete only ONE stage per approval cycle. Never advance without explicit approval.

1. **Analysis** → Wait for approval → **Requirements**
2. **Requirements** → Wait for approval → **ADR**
3. **ADR** → Wait for approval → **Task Package**
4. **Task Package** → Wait for approval → **Implementation**
5. **Implementation Phase N** → Wait for approval → **Phase N+1**

## Stage Details

### 1. Analysis
- Problem statement with context, alternatives, and recommendation
- Share draft and wait for approval before drafting requirements

### 2. Requirements
- Verifiable functional and non-functional requirements
- Derived from approved analysis
- Present requirements and wait for approval before writing ADR

### 3. Architecture Decision (ADR)
- Decision record describing structural approach
- Satisfies requirements
- Submit ADR and wait for approval before creating task package

### 4. Task Package
- Task directory with design.md, plan.md, README.md
- Linked to upstream artifacts
- Share task documents and wait for approval before starting implementation

### 5. Implementation
- Code and supporting assets tied to approved task
- **Each phase in plan.md is its own approval checkpoint**
- After finishing a phase:
  1. Stop immediately
  2. Mark checklist item as `[x]`
  3. Request explicit approval
  4. Do NOT proceed until approval received

## Exception Handling

### If Work Advanced Without Approval
- **Immediate Pause**: Stop all work
- **User Decision**: Ask whether to delete premature work and restart, or treat as draft for review
- **Upstream Gap**: If prerequisite artifact missing, suspend implementation, create missing document via template, secure approval, then resume

### If Instructions Are Ambiguous
- Treat as cue to request confirmation
- Do NOT interpret independently

## Status Maintenance

- Keep document metadata current
- Reviewers need to know the active phase
- Update traceability.md when adding/removing documents
