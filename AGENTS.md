PRs and docs in english

Every change updates the documentation in the same commit — see the `tyra-docs` skill for the checklist (README, the relevant skills, example READMEs).

Files that exist once per platform (`deps.ps1`/`deps.sh`, `setup.*`, `build.*`, the `if(WIN32)`/`else()` and `#ifdef _WIN32`/`#else` halves) are edited as a PAIR, in the same commit — see "Platform parity" in the `tyra-editor-dev` skill.

Respond in the same language as the User's message.
## Prune the test scratch root when done

`%TEMP%\tyra-editor-test` (pcsx2-capture slots, scratch projects) is never
cleaned and once filled a C: drive (3.4 GB, ~50 MB a capture slot). When you
finish testing, keep only its **10 most recently modified** entries and leave
any a running PCSX2 still uses. Never write a config that points INTO a slot
folder - pruning it breaks the config (docs/backlog.md, "Cap
`%TEMP%\tyra-editor-test`").
