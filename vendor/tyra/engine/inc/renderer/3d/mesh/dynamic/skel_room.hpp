#pragma once

// Added by TyraX: the question a render site with throwing containers asks before it builds
// (docs/memory-manager.md, "Render memory is requested, never demanded").
//
// The Skinning budget refuses a nothrow request when it is full and halts on a throwing one, and
// a std::vector, a make_shared or an unordered_map asks with throwing new. A site that has to use
// them first asks for the room they will take in ONE block from the nothrow operator new[] and
// gives it straight back: refused, the site makes nothing and the caller keeps the cheaper thing
// it has. The call is a call to the operator, not a new-expression: a compiler may drop a
// new-expression whose block is never used, and fold its null check to "not null".

#include <tamtypes.h>

#include <cstddef>
#include <new>

namespace Tyra {
namespace SkelSkin {

/** What the budget charges a block besides its bytes (the heap header and the rounding to 16),
 * for the sizes a probe adds up. */
constexpr size_t kBlockOverhead = 32;

/** The bytes of a std::vector of `n` elements of `elem` bytes, one block. */
constexpr size_t vectorBytes(size_t n, size_t elem) { return n * elem + kBlockOverhead; }

/** True when `bytes` can be had now. */
inline bool room(size_t bytes) {
  void* block = ::operator new[](bytes + kBlockOverhead, std::nothrow);
  if (block == nullptr) return false;
  ::operator delete[](block);
  return true;
}

}  // namespace SkelSkin
}  // namespace Tyra
