/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the cull TD program that reads INDEXED corners
# (docs/animated-models.md, "Skinned streams with indexed corners").
*/

#include "debug/debug.hpp"
#include "packet2/packet2_tyra_utils.hpp"
#include "renderer/3d/pipeline/static/core/programs/cull/stapip_cull_td_indexed_vu1_program.hpp"

// The folded-light image: the default lit program's light form (stapip_vu1_experiments.hpp,
// TYRA_VU1_EXP_EE_LIGHT_FOLD), which is the only form the indexed image is built for.
extern u32 StaPipVU1Cull_TD_Indexed_FOLD_CodeStart __attribute__((section(".vudata")));
extern u32 StaPipVU1Cull_TD_Indexed_FOLD_CodeEnd __attribute__((section(".vudata")));

namespace Tyra {

// The same GS registers, vertex width and element count as the plain cull TD (its output is
// word for word the same).
StaPipCullTDIndexedVU1Program::StaPipCullTDIndexedVU1Program()
    : StaPipVU1Program(StaPipCullTextureDirLightsIndexed,
                       &StaPipVU1Cull_TD_Indexed_FOLD_CodeStart,
                       &StaPipVU1Cull_TD_Indexed_FOLD_CodeEnd,
                       ((u64)GIF_REG_ST) << 0 | ((u64)GIF_REG_RGBAQ) << 4 |
                           ((u64)GIF_REG_XYZF2) << 8,
                       3, 4) {}

StaPipCullTDIndexedVU1Program::~StaPipCullTDIndexedVU1Program() {}

std::string StaPipCullTDIndexedVU1Program::getStringName() const {
  return std::string("StaPip - Cull - TD - indexed");
}

void StaPipCullTDIndexedVU1Program::addProgramQBufferDataToPacket(
    packet2_t*, StaPipQBuffer*) const {
  // An indexed buffer's two REFs are emitted by the renderer; this program has no compact runs.
  TYRA_TRAP("The indexed cull TD program's data is sent by StaPipQBufferRenderer");
}

}  // namespace Tyra
