#pragma once

// The skin's influence limit (TYRA_SKIN_MAX_INFLUENCES, 1..4; 4 = every influence kept). A
// corner keeps its K largest weights (ties: the lower joint id first) and the kept weights are
// rescaled to sum to 255 by the largest-remainder rule. A corner with K or fewer nonzero
// weights is unchanged. The loader applies it to every mesh of a model, so the skin, its
// stream oracle and the host references all read the same truncated weights.

#include "loaders/3d/tskl_loader/tskl_loader.hpp"

#ifndef TYRA_SKIN_MAX_INFLUENCES
#define TYRA_SKIN_MAX_INFLUENCES 4
#endif

namespace Tyra {

/** Limits one corner's four slots (joints j[4], weights w[4]) to its `k` largest weights. */
inline void limitCornerInfluences(u8* j, u8* w, u32 k) {
  u32 nonzero = 0;
  for (int s = 0; s < 4; s++) nonzero += w[s] ? 1U : 0U;
  if (k >= 4 || nonzero <= k || k == 0) return;
  // The slots by weight (descending), ties by joint id (ascending): insertion sort.
  int order[4] = {0, 1, 2, 3};
  for (int a = 1; a < 4; a++)
    for (int b = a; b > 0; b--) {
      const int p = order[b - 1], q = order[b];
      const bool before = w[q] > w[p] || (w[q] == w[p] && j[q] < j[p]);
      if (!before) break;
      order[b - 1] = q;
      order[b] = p;
    }
  u32 sum = 0;
  for (u32 i = 0; i < k; i++) sum += w[order[i]];
  // Rescale the kept weights to 255 (largest remainder), written back to their own slots.
  u32 out[4] = {0, 0, 0, 0}, rem[4] = {0, 0, 0, 0}, total = 0;
  for (u32 i = 0; i < k; i++) {
    const u32 num = (u32)w[order[i]] * 255U;
    out[i] = num / sum;
    rem[i] = num % sum;
    total += out[i];
  }
  while (total < 255U) {
    u32 best = 0;
    for (u32 i = 1; i < k; i++)
      if (rem[i] > rem[best]) best = i;
    out[best]++;
    rem[best] = 0;
    total++;
  }
  u8 nj[4] = {0, 0, 0, 0}, nw[4] = {0, 0, 0, 0};
  for (u32 i = 0; i < k; i++) {
    const int s = order[i];
    nj[s] = j[s];
    nw[s] = (u8)out[i];
  }
  for (int s = 0; s < 4; s++) {
    j[s] = nj[s];
    w[s] = nw[s];
  }
}

/** Limits every corner of one mesh (joints and weights, four slots a corner). */
inline void limitMeshInfluences(SkelMesh& m, u32 k) {
  if (k >= 4) return;
  for (u32 v = 0; v < m.vertexCount; v++)
    limitCornerInfluences(&m.joints[(size_t)v * 4], &m.weights[(size_t)v * 4], k);
}

}  // namespace Tyra
