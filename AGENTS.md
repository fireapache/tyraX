PRs and docs in english

Every change updates the documentation in the same commit — see the `tyra-docs` skill for the checklist (README, the relevant skills, example READMEs).

Files that exist once per platform (`deps.ps1`/`deps.sh`, `setup.*`, `build.*`, the `if(WIN32)`/`else()` and `#ifdef _WIN32`/`#else` halves) are edited as a PAIR, in the same commit — see "Platform parity" in the `tyra-editor-dev` skill.

Respond in the same language as the User's message.

## Namespaces for game-side helpers

Game-side helpers that ship in a release build live in **`Tyra::helper`**
(`vendor/tyra/engine/inc/physics/ray.hpp`) â€” trace and sweep helpers:
`Tyra::helper::traceLine(start, end)`, `setTraceWorld(...)`,
`setTraceBroadPhase(...)` (the contact broad phase), `TraceHit`. New
trace/sweep/overlap helpers go here.
