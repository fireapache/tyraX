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

#pragma once

#include <packet2_utils.h>
#include <string>
#include "../../stapip_vu1_program.hpp"

namespace Tyra {

/** The folded-light cull TD program with a corner table in front of the unique vertices: the
 * package is two REFs (the body's pose block, the model's static block), which the renderer
 * emits itself (StaPipQBufferRenderer::addBufferCommands), so this wrapper has no runs to send. */
class StaPipCullTDIndexedVU1Program : public StaPipVU1Program {
 public:
  StaPipCullTDIndexedVU1Program();
  ~StaPipCullTDIndexedVU1Program();

  std::string getStringName() const;
  void addProgramQBufferDataToPacket(packet2_t* packet,
                                     StaPipQBuffer* qbuffer) const;
};

}  // namespace Tyra
