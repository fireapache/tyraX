/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Sandro Sobczyński <sandro.sobczynski@gmail.com>
*/

#include "debug/debug.hpp"
#include "packet2/packet2_tyra_utils.hpp"
#include "renderer/3d/pipeline/static/core/programs/cull/stapip_cull_d_vu1_program.hpp"

#include "renderer/3d/pipeline/static/core/stapip_vu1_experiments.hpp"

// Modified by TyraX: a measured-only experiment can swap in another image
// (stapip_vu1_experiments.hpp). The #if branch is the experiment; the
// #else branch is the shipping image `--vu-check` verifies.
#if TYRA_VU1_EXP_EE_LIGHT_FOLD
extern u32 StaPipVU1Cull_D_FOLD_CodeStart __attribute__((section(".vudata")));
extern u32 StaPipVU1Cull_D_FOLD_CodeEnd __attribute__((section(".vudata")));
#define TYRA_WRAPPER_IMAGE_START (&StaPipVU1Cull_D_FOLD_CodeStart)
#define TYRA_WRAPPER_IMAGE_END (&StaPipVU1Cull_D_FOLD_CodeEnd)
#else
extern u32 StaPipVU1Cull_D_CodeStart __attribute__((section(".vudata")));
extern u32 StaPipVU1Cull_D_CodeEnd __attribute__((section(".vudata")));
#define TYRA_WRAPPER_IMAGE_START (&StaPipVU1Cull_D_CodeStart)
#define TYRA_WRAPPER_IMAGE_END (&StaPipVU1Cull_D_CodeEnd)
#endif

namespace Tyra {

StaPipCullDVU1Program::StaPipCullDVU1Program()
    : StaPipVU1Program(StaPipCullDirLights, TYRA_WRAPPER_IMAGE_START,
                       TYRA_WRAPPER_IMAGE_END,
                       ((u64)GIF_REG_RGBAQ) << 0 | ((u64)GIF_REG_XYZF2) << 4, 2,
                       3) {}

StaPipCullDVU1Program::~StaPipCullDVU1Program() {}

std::string StaPipCullDVU1Program::getStringName() const {
  return std::string("StaPip - Cull - D");
}

void StaPipCullDVU1Program::addProgramQBufferDataToPacket(
    packet2_t* packet, StaPipQBuffer* qbuffer) const {
  u32 addr = VU1_STAPIP_VERT_DATA_ADDR;
  // Compact input (docs/compact-vertex-format.md): each block is a
  // run - a header quadword (STMASK, mask, STMOD, UNPACK) and its
  // payload - sent with one REF. The header names its own VU1
  // address, which must be where this layout puts the block.

  // Vertices
  TYRA_ASSERT((static_cast<const u32*>(qbuffer->run[0])[3] & 0x3FF) == addr,
              "compact run 0 unpacks to the wrong VU1 address");
  Packet2TyraUtils::addCompactRun(packet, qbuffer->run[0],
                                  qbuffer->runQwords[0]);

  // Normals
  addr += qbuffer->size;
  TYRA_ASSERT((static_cast<const u32*>(qbuffer->run[1])[3] & 0x3FF) == addr,
              "compact run 1 unpacks to the wrong VU1 address");
  Packet2TyraUtils::addCompactRun(packet, qbuffer->run[1],
                                  qbuffer->runQwords[1]);
  (void)addr;  // read by the asserts only
}

}  // namespace Tyra
