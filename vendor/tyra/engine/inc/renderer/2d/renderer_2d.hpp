/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Sandro Sobczyński <sandro.sobczynski@gmail.com>
*/

#pragma once

#include "renderer/core/2d/sprite/sprite.hpp"
#include "renderer/core/renderer_core.hpp"

namespace Tyra {

class Renderer2D {
 public:
  Renderer2D();
  ~Renderer2D();

  void init(RendererCore* rendererCore);
  void render(const Sprite* sprite);
  void render(const Sprite& sprite);

  /**
   * Modified by TyraX: queue a game's own finished GIF packet (3D overlay
   * lines, flat triangles - register writes and primitives, ending on an EOP
   * tag) behind the 3D on the VIF1 chain the sprites ride, so the EE never
   * waits for VU1 to finish the scene before it can draw over it. Copied; the
   * caller may reuse its buffer. False - send it over PATH3 yourself, after
   * draining the 3D as the stock way does - when the chain is off
   * (TYRA_2D_VIF1_DIRECT 0), VU1 is not up yet, or the packet is bigger than
   * a chain. docs/ee-submission-rearchitecture.md, "Game GIF packets on the
   * chain".
   */
  bool queueGifPacket(const qword_t* data, u32 qwc);

 private:
  RendererCore* core;
};

}  // namespace Tyra
