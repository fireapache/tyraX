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

// Added by TyraX: the memory manager's registry - the one place that says which
// budgets and which pools a game has and how big each is. The engine registers
// its own rows; the game overrides them and adds its own (ids from
// Budget::FirstGame up). From the rows the registry computes how much fixed
// memory the game needs, where each piece of it goes (Layout) and whether all
// of it fits the RAM there is (fits), so a registry that is too big is found at
// boot with the table printed, not hours later as a black screen.
//
// The registry only describes and computes. It allocates nothing and does not
// call fail(): fits() answers false and error() says why; whoever started the
// manager decides what to do about it.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>

namespace Tyra {
namespace Memory {

using BudgetId = uint16_t;
using PoolId = uint16_t;

enum class Kind : uint8_t { System, Logic, Render, Asset, Scratch };

// What a pool does when it is full and its budget cannot take another
// extension.
enum class Policy : uint8_t { Refuse, RecycleOldest, Halt };

// What a budget does with a request it has no room for (Manager::allocate).
// Halt: the request is a failure ("budget full") and the game stops - memory
// whose running out is a bug (logic, the system, scratch). Refuse: the request
// answers null and the caller does without - render data and assets are
// requests: refused, the renderer keeps what it has, draws something cheaper
// or skips it this frame, and asks again the next.
enum class OnFull : uint8_t { Halt, Refuse };

// Refuse for Kind::Render and Kind::Asset, Halt for every other kind.
constexpr OnFull defaultOnFull(Kind k) {
  return k == Kind::Render || k == Kind::Asset ? OnFull::Refuse : OnFull::Halt;
}

// The engine's budgets are the first ids; a game's start at FirstGame.
namespace Budget {
constexpr BudgetId System = 0, FrameScratch = 1, LoadScratch = 2, Renderer = 3,
                   Skinning = 4, BakedStreams = 5, Audio = 6, Level = 7,
                   FirstGame = 32, Max = 64;
}  // namespace Budget

constexpr int kMaxPools = 64;

// pool() answers this when the table is full (fits() then reports it).
constexpr PoolId kNoPool = 0xFFFF;

struct BudgetDesc {
  const char* name;  // static text: the registry keeps the pointer
  Kind kind;
  // The budget's own region (a scratch's size). 0: it has no region and is
  // served from `System`'s, charged to itself and uncapped - measure mode.
  // Handed to RegionHeap::init, so keep it a multiple of 16.
  uint32_t regionBytes;
  // How much of the movable heap this budget's pools may take as extensions.
  uint32_t extensionBytes;
  // What a request it has no room for does. budget() sets it from the kind
  // (defaultOnFull) whenever it names a row; a game may change it after.
  OnFull onFull;
};

struct PoolDesc {
  const char* name;  // static text
  BudgetId budget;   // must be a registered budget
  uint32_t typeSize;
  uint32_t align;       // a power of two (0 or 1: no demand)
  uint32_t firstCount;  // entries in the first bucket, which is fixed memory
  uint32_t extensionCount;  // entries in each extension bucket
  uint32_t maxExtensions;   // how many extension buckets the pool may take
  Policy policy;
};

struct Settings {
  // The entries of the Ptr table (PtrTable): a power of two, or 0 for none.
  uint32_t ptrTableCapacity = 4096;
  uint32_t defragBytesPerFrame = 64 * 1024;
  // What the C library keeps for itself, left over after the block.
  uint32_t newlibReserveBytes = 256 * 1024;
  // The size of the system region; 0: everything that is left.
  uint32_t systemBytes = 0;
  // A refusing budget with no region of its own (measure mode: served from
  // System's region) also refuses a request that would leave System's region
  // with less than this free, so render data can never starve the game's
  // logic of the memory it halts without.
  uint32_t refuseHeadroomBytes = 512 * 1024;
  // The frames between two lines about one refusing call site (or one asset a
  // render site did without): a second at 60 frames a second.
  uint32_t refusalReportFrames = 60;
};

// Where each fixed piece sits, as byte offsets from the start of the one block
// that holds them all. Every piece starts on a multiple of 64 (a pool whose
// type is aligned to more starts on a multiple of that); the block itself must
// be allocated aligned to blockAlign.
//
// regionOffset[id] is meaningful for a budget with regionBytes != 0 and
// poolOffset[id] for a registered pool; the rest is 0.
struct Layout {
  // What the registry needs up front: everything before the system region plus
  // the system region when it was given a size (Settings::systemBytes); when it
  // takes what is left, its size is not part of it.
  size_t fixedBytes;
  size_t systemBytes;            // the size of the system region
  size_t heapOffset, heapBytes;  // the movable heap
  size_t ptrOffset, ptrBytes;    // the Ptr table's storage
  size_t regionOffset[Budget::Max];
  size_t poolOffset[kMaxPools];  // each pool's first bucket
  size_t systemOffset;           // the system region: last in the block
  size_t blockBytes;             // the whole block: systemOffset + systemBytes
  size_t blockAlign;  // 64, or the largest alignment a first bucket asks for
};

// How one bucket of a pool is laid out, as byte offsets from the bucket's base.
// There is one formula for it (Registry::bucketParts); whoever fills or reads a
// bucket - the layout, a pool - reads the offsets from here.
//
// A part ends where the next begins: the entries take [entries, bitmap), the
// live bitmap [bitmap, stamps), the recycle stamps [stamps, generations) and
// the generations [generations, total). A part a pool does not have (stamps
// unless the policy is RecycleOldest, generations unless TYRA_MEMORY_CHECKS) is
// zero-sized, but its offset is still filled in. Every offset is a multiple of
// 16. All zero for a pool the registry never issued.
struct BucketParts {
  size_t entries;      // always 0: the entries come first
  size_t bitmap;       // a bit per entry, set while the entry is live
  size_t stamps;       // 4 bytes per entry, RecycleOldest only
  size_t generations;  // 2 bytes per entry, TYRA_MEMORY_CHECKS only
  size_t total;        // the whole bucket: what bucketBytes answers
  uint32_t stride;     // bytes from one entry to the next
};

class Registry {
 public:
  using BucketParts = Tyra::Memory::BucketParts;

  // Registers the engine rows - System, FrameScratch, LoadScratch, Renderer,
  // Skinning, BakedStreams, Audio, Level - every one with regionBytes 0 and
  // extensionBytes 0, so the engine's default is one heap, as without a
  // manager, and the onFull of its kind.
  Registry();

  // The row for `id`, to override. An id that is not registered yet is added. A
  // non-null `name` (re)declares the row's name and kind, and its onFull to the
  // kind's (defaultOnFull); null leaves an existing row alone (and names a new
  // one "unnamed", a Logic row that halts). An id past Budget::Max is a
  // registration error that fits() reports; the row returned then is a dummy.
  BudgetDesc& budget(BudgetId id, const char* name = nullptr,
                     Kind kind = Kind::Logic);

  // The row for `id`, or null when it is not registered.
  const BudgetDesc* findBudget(BudgetId id) const;

  // Adds a pool; ids are dense from 0. kNoPool (and an error for fits() to
  // report) when kMaxPools pools exist already.
  PoolId pool(const PoolDesc& d);
  uint32_t poolCount() const;
  const PoolDesc& poolDesc(PoolId id) const;  // an empty row for an unknown id

  Settings settings;

  // Computes the layout of `availableBytes` of RAM into `out` (filled even when
  // the answer is false, so the table can be printed) and checks it. False,
  // with error() set, when a pool names a budget that is not registered, a
  // pool's alignment is not a power of two or is above 128 for a pool that
  // takes extensions from the movable heap, a registration call went wrong,
  // ptrTableCapacity is not a power of two (or 0), or fixedBytes +
  // newlibReserveBytes is more than availableBytes. Never calls fail().
  bool fits(size_t availableBytes, Layout& out) const;

  // Why the last fits() answered false; "" after a fits() that answered true or
  // before the first one.
  const char* error() const;

  // The layout of one bucket of `count` entries of the pool: the entries, the
  // live bitmap, the recycle stamps of a RecycleOldest pool and, with
  // TYRA_MEMORY_CHECKS, the generations. All zero for an unknown pool.
  BucketParts bucketParts(PoolId id, uint32_t count) const;
  // The bytes of one such bucket: bucketParts(id, count).total. 0 for an
  // unknown pool.
  size_t bucketBytes(PoolId id, uint32_t count) const;
  // The pool's first bucket: bucketBytes(id, firstCount).
  size_t poolFixedBytes(PoolId id) const;

  // One `print` call per line, without a newline: a line per budget, a line per
  // pool, then the totals of `layout`.
  void printTable(void (*print)(const char*), const Layout& layout) const;

 private:
  BudgetDesc budgets_[Budget::Max];
  bool known_[Budget::Max];
  PoolDesc pools_[kMaxPools];
  uint32_t poolCount_;
  BudgetDesc dummy_;
  const char* registrationError_;
  mutable char error_[192];
};

}  // namespace Memory
}  // namespace Tyra
