/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the indexed skinned stream's build switch.
*/

#pragma once

// The indexed skinned stream's switch (docs/animated-models.md, "Skinned streams with indexed
// corners"). 1 (the default) skins each unique vertex once into a pose buffer per body and lets
// VU1 assemble the triangles by index; 0 keeps the plain corner streams. Compile-time and
// engine-wide: it changes the VU1 program set the renderer uploads, the triangle order the
// skinned models are bound in and what the game template asks the engine for, so the engine
// library and the game must be built with the same value - checkBuild() below makes a
// mismatch fail to link instead of corrupting the packet layout.
#ifndef TYRA_SKIN_INDEXED
#define TYRA_SKIN_INDEXED 1
#endif

namespace Tyra {
namespace SkelSkin {
namespace Indexed {

/** The build guard. The library defines one of two symbols for the value it was built with
 * (skel_indexed_config.cpp); checkBuild() reads the symbol of the value of the code that calls it
 * (the game template at start up), so a game and a library built with different
 * TYRA_SKIN_INDEXED values fail at link time with an undefined `kBuildIndexed...`. */
#if TYRA_SKIN_INDEXED
extern const unsigned char kBuildIndexedOn;
inline unsigned char checkBuild() { return *static_cast<const volatile unsigned char*>(&kBuildIndexedOn); }
#else
extern const unsigned char kBuildIndexedOff;
inline unsigned char checkBuild() { return *static_cast<const volatile unsigned char*>(&kBuildIndexedOff); }
#endif

/** The corners a package holds in an indexed stream: the indexed VU1 program's ceiling (the
 * plain cull TD package). The game template builds the skinned bags' stream layouts with it. */
constexpr unsigned kStreamPackage = 75;
/** The package size a part falls back to when 75-corner packages cannot be built for it. */
constexpr unsigned kFallbackPackage = 48;

}  // namespace Indexed
}  // namespace SkelSkin
}  // namespace Tyra
