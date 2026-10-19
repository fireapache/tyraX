PRs and docs in english

Every change updates the documentation in the same commit — see the `tyra-docs` skill for the checklist (README, the relevant skills, example READMEs).

Files that exist once per platform (`deps.ps1`/`deps.sh`, `setup.*`, `build.*`, the `if(WIN32)`/`else()` and `#ifdef _WIN32`/`#else` halves) are edited as a PAIR, in the same commit — see "Platform parity" in the `tyra-editor-dev` skill.

Respond in the same language as the User's message.

## Namespaces for game-side helpers

Two homes inside the engine namespace, and the split is about whether the code
ships:

- **`Tyra::helper`** (`vendor/tyra/engine/inc/physics/ray.hpp`) — trace and
  sweep helpers that ship: `Tyra::helper::traceLine(start, end)`,
  `setTraceWorld(...)`, `setTraceBroadPhase(...)` (the contact broad phase), `TraceHit`. New trace/sweep/overlap helpers go here.
- **`Tyra::debug`** (`vendor/tyra/engine/inc/debug/debug_draw.hpp`) — **every
  function that is compiled away in a release build goes here**:
  `Tyra::debug::drawLine(from, to, color = green, time = 0)`, and the shapes
  that come after it. The whole header is `#ifndef NDEBUG` with no-op overloads
  in release, so call sites need no `#ifdef`.

Same pattern as the rest of the engine (`Tyra::Vec4`, `Tyra::Renderer`); the
sub-namespace is what says whether a shipping binary has the code. A debug-only
verb left loose in `Tyra` reads like one that ships, which is the mistake this
rule prevents.
## Prune the test scratch root when done

`%TEMP%\tyra-editor-test` (pcsx2-capture slots, scratch projects) is never
cleaned and once filled a C: drive (3.4 GB, ~50 MB a capture slot). When you
finish testing, keep only its **10 most recently modified** entries and leave
any a running PCSX2 still uses. Never write a config that points INTO a slot
folder - pruning it breaks the config (docs/backlog.md, "Cap
`%TEMP%\tyra-editor-test`").
