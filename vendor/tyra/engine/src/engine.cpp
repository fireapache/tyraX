/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Sandro Sobczyński <sandro.sobczynski@gmail.com>
# Modified by TyraX: demote the main thread below the audio threads;
#                    USB keyboard/mouse device (kbdMouse);
#                    the memory manager's start, frame start and scopes
*/

#include "debug/hardware_trace.hpp"
#include "engine.hpp"
#include "debug/sifrpc_guard.hpp"
#include <kernel.h>
#if TYRA_MEMORY_MANAGER
#include "memory/manager.hpp"
#endif

namespace Tyra {

#if TYRA_MEMORY_MANAGER
namespace {

// Added by TyraX: the memory manager (docs/memory-manager.md). Before the
// engine allocates anything of its own: the EE binding, the game's registry
// on top of the engine's rows, the one block, and the table in the log.
void startMemoryManager() {
  Memory::install();
  Memory::Registry registry;
  Memory::registerGameBudgets(registry);
  Memory::Manager& manager = Memory::Manager::instance();
  if (manager.start(registry)) manager.printTable();
}

}  // namespace
#endif

Engine::Engine() {
#if TYRA_MEMORY_MANAGER
  startMemoryManager();
#endif
  initAll(EngineOptions());
}

Engine::Engine(const EngineOptions& options) {
  // First: the manager's lines go where the log goes.
  info.writeLogsToFile = options.writeLogsToFile;
#if TYRA_MEMORY_MANAGER
  startMemoryManager();
#endif
  initAll(options);
}

Engine::~Engine() {}

void Engine::run(Game* t_game) {
  game = t_game;
  game->init();
  HardwareTrace::configure();
  while (true) {
    realLoop();
  }
}

void Engine::realLoop() {
  HardwareTrace::beginFrame();
#if TYRA_MEMORY_MANAGER
  {
    HardwareTrace::Scope trace("Memory", HardwareTrace::Kind::Span, false);
    Memory::Manager::instance().frameStart();
    Memory::publishTelemetry();
  }
#endif
  { HardwareTrace::Scope trace("Pad", HardwareTrace::Kind::Span, false); pad.update(); }
  if (kbdMouse.isEnabled()) kbdMouse.update();
  { HardwareTrace::Scope trace("Game", HardwareTrace::Kind::Span, false); game->loop(); }
  { HardwareTrace::Scope trace("Info", HardwareTrace::Kind::Span, false); info.update(); }
  // One compare per frame unless the guard has actually dropped something.
  SifRpcGuard::report();
  HardwareTrace::endFrame();
}

void Engine::initAll(const EngineOptions& options) {
  // The audio thread (0x5) and the song streamer (0x6) must be able to
  // PREEMPT the game: depending on the boot path the main thread arrives
  // here at priority 0 (nothing preempts it) or whatever the loader used,
  // and the engine's GS waits are busy-spins that never yield the CPU. A
  // compute-bound frame then starves the audio threads and streamed music
  // audibly drags exactly when the game dips below full frame rate. The
  // audio threads cost microseconds per wake, so the game loses nothing.
  //
  // The value must stay numerically ABOVE 20: ps2link's EE command thread
  // (the one that services "ps2client reset"/execee while a game runs) is
  // priority 20, and the EE scheduler is strictly priority-based. An earlier
  // 0x10 here starved it forever - the first network deploy worked, but Stop
  // and every redeploy hung because the console never processed the reset.
  ChangeThreadPriority(GetThreadId(), 0x40);

  // Before ANY file access: the guard replaces ps2sdk's SIF_CMD_RPC_END
  // handler, which crashes the EE on a duplicate completion instead of
  // ignoring it (inc/debug/sifrpc_guard.hpp). It goes here rather than in
  // IrxLoader because IrxLoader's ctor is what calls SifInitRpc(), and that
  // is what installs the handler this replaces.
  SifRpcGuard::install();

  srand(time(nullptr));
  // No keyboard/mouse under ps2link by default: USB drivers cannot be added
  // safely to a ps2link that is already running (its IOP is never reset, and
  // ps2kbd/ps2mouse need a usbd that a network-booted ps2link does not carry).
  // The loadUsbKbdMouseUnderPs2Link override targets the TyraX ps2link
  // (tools/ps2link), which bakes usbd + ps2kbd + ps2mouse into its OWN boot:
  // we then reuse that resident stack and load nothing of our own - a second
  // usbd would wedge it. The editor deploys to no other ps2link, so it turns
  // the override on by default (docs/ps2link-setup.md).
  const bool underPs2Link = IrxLoader::keepIopResident;
  const bool withKbdMouse =
      options.loadUsbKbdMouse &&
      (!underPs2Link || options.loadUsbKbdMouseUnderPs2Link);
  const bool loadOwnHid = withKbdMouse && !underPs2Link;
  irx.loadAll(options.loadUsbDriver, loadOwnHid, info.writeLogsToFile);
  // Modified by TyraX: the renderer's init-time knobs travel as a struct.
  RendererOptions rendererOptions;
  rendererOptions.videoMode = options.videoMode;
  rendererOptions.displayMode = options.displayMode;
  rendererOptions.widescreen = options.widescreen;
  rendererOptions.colorDepth = options.colorDepth;
  rendererOptions.dither = options.dither;
  rendererOptions.envMap = options.envMapTarget;
  rendererOptions.envMapDoubleBuffered = options.envMapDoubleBuffered;
  rendererOptions.camFeed = options.camFeedTarget;
  rendererOptions.tripleBuffering = options.tripleBuffering;
  {
#if TYRA_MEMORY_MANAGER
    Memory::MemScope scope(Memory::Budget::Renderer);
#endif
    renderer.init(rendererOptions);
  }
  // Modified by TyraX: the logo can be switched off (EngineOptions).
  if (options.showBanner) banner.show(&renderer);
  {
#if TYRA_MEMORY_MANAGER
    Memory::MemScope scope(Memory::Budget::Audio);
#endif
    audio.init();
  }
  pad.init();
  if (withKbdMouse) kbdMouse.init(underPs2Link);
}

}  // namespace Tyra
