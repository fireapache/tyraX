PRs and docs in english

Every change updates the documentation in the same commit — see the `tyra-docs` skill for the checklist (README, the relevant skills, example READMEs).

Files that exist once per platform (`deps.ps1`/`deps.sh`, `setup.*`, `build.*`, the `if(WIN32)`/`else()` and `#ifdef _WIN32`/`#else` halves) are edited as a PAIR, in the same commit — see "Platform parity" in the `tyra-editor-dev` skill.

Respond in the same language as the User's message.

## Namespaces for game-side helpers

Two homes, and the split is about whether the code ships:

- **`tyra::helper`** (`vendor/tyra/engine/inc/physics/ray.hpp`) — trace and
  sweep helpers that ship: `tyra::helper::traceLine(start, end)`,
  `setTraceWorld(...)`, `TraceHit`. New trace/sweep/overlap helpers go here.
- **`tyra::debug`** (`vendor/tyra/engine/inc/debug/debug_draw.hpp`) — **every
  function that is compiled away in a release build goes here**:
  `tyra::debug::drawLine(from, to, color = green, time = 0)`, and the shapes
  that come after it. The whole header is `#ifndef NDEBUG` with no-op overloads
  in release, so call sites need no `#ifdef`.

Both are lowercase `tyra` and deliberately NOT the engine's `Tyra` (which holds
engine types like `Tyra::Vec4`). A debug-only function sitting in `Tyra` reads
as shipping code, which is the mistake this rule exists to prevent. Do not
"fix" the lowercase.