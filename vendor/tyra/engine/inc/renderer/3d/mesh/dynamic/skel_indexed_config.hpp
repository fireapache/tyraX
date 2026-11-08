#pragma once

// The indexed skinned stream's switch (docs/animated-models.md, "Skinned streams with indexed
// corners"). 1 (the default) skins each unique vertex once into a pose buffer per body and lets
// VU1 assemble the triangles by index; 0 keeps the plain corner streams. Compile-time and
// engine-wide: it changes the VU1 program set the renderer uploads, the triangle order the
// skinned models are bound in and what the game template asks the engine for, so the engine
// library and the game must be built with the same value.
#ifndef TYRA_SKIN_INDEXED
#define TYRA_SKIN_INDEXED 1
#endif

namespace Tyra {
namespace SkelSkin {
namespace Indexed {

/** The corners a package holds in an indexed stream: the indexed VU1 program's ceiling (the
 * plain cull TD package). The game template builds the skinned bags' stream layouts with it. */
constexpr unsigned kStreamPackage = 75;
/** The package size a part falls back to when 75-corner packages cannot be built for it. */
constexpr unsigned kFallbackPackage = 48;

}  // namespace Indexed
}  // namespace SkelSkin
}  // namespace Tyra
