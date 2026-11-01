/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# TyraX
#-----------------------------------------------------------------------
*/

// Added by TyraX: the memory manager's umbrella header - what a game includes
// to declare its budgets and pools (Registry), open budget scopes (MemScope),
// use pools (Pool<T>, Ptr<T>), the per-frame scratch (Scratch::vector, ...),
// read the stats (Manager) and handle a failure (setFailureHandler). See
// docs/memory-manager.md.

#pragma once

#include "memory/failure.hpp"
#include "memory/manager.hpp"
#include "memory/pool.hpp"
#include "memory/ptr.hpp"
#include "memory/registry.hpp"
#include "memory/scratch.hpp"
