/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the library side of the TYRA_SKIN_INDEXED build guard.
*/

// The one symbol that says which TYRA_SKIN_INDEXED value the engine library was built with
// (skel_indexed_config.hpp, checkBuild): the game names the symbol of ITS value, so a game
// and a library built with different values do not link.

#include "renderer/3d/mesh/dynamic/skel_indexed_config.hpp"

namespace Tyra {
namespace SkelSkin {
namespace Indexed {

#if TYRA_SKIN_INDEXED
const unsigned char kBuildIndexedOn = 1;
#else
const unsigned char kBuildIndexedOff = 1;
#endif

}  // namespace Indexed
}  // namespace SkelSkin
}  // namespace Tyra
