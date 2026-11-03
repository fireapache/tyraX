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
#include "renderer/3d/pipeline/static/core/programs/cull/stapip_cull_tc_vu1_program.hpp"

extern u32 StaPipVU1Cull_TC_CodeStart __attribute__((section(".vudata")));
extern u32 StaPipVU1Cull_TC_CodeEnd __attribute__((section(".vudata")));
#define TYRA_WRAPPER_IMAGE_START (&StaPipVU1Cull_TC_CodeStart)
#define TYRA_WRAPPER_IMAGE_END (&StaPipVU1Cull_TC_CodeEnd)

namespace Tyra {

StaPipCullTCVU1Program::StaPipCullTCVU1Program()
    : StaPipVU1Program(StaPipCullTextureColor, TYRA_WRAPPER_IMAGE_START,
                       TYRA_WRAPPER_IMAGE_END,
                       ((u64)GIF_REG_ST) << 0 | ((u64)GIF_REG_RGBAQ) << 4 |
                           ((u64)GIF_REG_XYZF2) << 8,
                       3, 3) {}

StaPipCullTCVU1Program::~StaPipCullTCVU1Program() {}

std::string StaPipCullTCVU1Program::getStringName() const {
  return std::string("StaPip - Cull - TC");
}

void StaPipCullTCVU1Program::addProgramQBufferDataToPacket(
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

  // ST
  addr += qbuffer->size;
  TYRA_ASSERT((static_cast<const u32*>(qbuffer->run[1])[3] & 0x3FF) == addr,
              "compact run 1 unpacks to the wrong VU1 address");
  Packet2TyraUtils::addCompactRun(packet, qbuffer->run[1],
                                  qbuffer->runQwords[1]);

  // Colors - absent when the mesh draws in a single colour
  if (qbuffer->bag->color->single == nullptr) {
    addr += qbuffer->size;
    TYRA_ASSERT((static_cast<const u32*>(qbuffer->run[2])[3] & 0x3FF) == addr,
                "compact run 2 unpacks to the wrong VU1 address");
    Packet2TyraUtils::addCompactRun(packet, qbuffer->run[2],
                                    qbuffer->runQwords[2]);
  }
  (void)addr;  // read by the asserts only
}

}  // namespace Tyra
