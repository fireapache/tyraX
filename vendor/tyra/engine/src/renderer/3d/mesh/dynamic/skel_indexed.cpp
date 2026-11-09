// The skinned compact stream with indexed corners (skel_indexed.hpp): plain C++, run by the
// host verifier and, later, by the EE.

#include "renderer/3d/mesh/dynamic/skel_indexed.hpp"

#include "renderer/3d/mesh/dynamic/skel_room.hpp"

#include <algorithm>
#include <cstring>

namespace Tyra {
namespace SkelSkin {
namespace Indexed {

namespace {

constexpr u32 kNop = 0u;
constexpr u32 kStmask = 0x20000000u;
constexpr u32 kStmod = 0x05000000u;
constexpr u32 kPosMask = 0x80808080u;  // w from COL (1.0), as every compact position run

// An UNPACK VIFcode by hand (as Compact::unpackCode: the SDK helpers set USN): vn/vl nibble
// `fmt`, `masked` sets the m bit, signed data, TOPS-relative (FLG).
u32 unpack(Compact::Format fmt, bool masked, u32 num, u32 dest) {
  const u32 cmd = 0x60u | (masked ? 0x10u : 0u) | static_cast<u32>(fmt);
  return (cmd << 24) | ((num & 0xFFu) << 16) | (dest & 0x3FFu) | 0x8000u;
}

void putWords(u8* at, u32 a, u32 b, u32 c, u32 d) {
  const u32 w[4] = {a, b, c, d};
  std::memcpy(at, w, 16);
}

}  // namespace

bool build(const u32* skinSource, u32 corners, u32 packageSize, Level& out, bool* refused) {
  out = Level();
  if (refused != nullptr) *refused = false;
  if (corners == 0 || packageSize < 3 || packageSize > kMaxPackage || packageSize % 3 != 0)
    return false;
  out.corners = corners;
  out.packageSize = packageSize;
  out.packages = (corners + packageSize - 1) / packageSize;
  // The first stage's tables, asked for at their exact sizes (the uniques are counted first).
  u32 counted = 0;
  for (u32 v = 0; v < corners; ++v)
    if (skinSource[v] == v) ++counted;
  if (!room(vectorBytes(corners, sizeof(u32)) + vectorBytes(corners, sizeof(u16)) +
            vectorBytes(out.packages + 1, sizeof(u32)) + vectorBytes(counted, sizeof(u16)) +
            vectorBytes(counted, sizeof(u32)))) {
    out = Level();
    if (refused != nullptr) *refused = true;
    return false;
  }
  out.uid.assign(corners, 0);
  u32 uniques = 0;
  for (u32 v = 0; v < corners; ++v)
    if (skinSource[v] == v) out.uid[v] = uniques++;
    else out.uid[v] = out.uid[skinSource[v]];
  out.uniques = uniques;
  out.local.assign(corners, 0);
  out.first.assign(out.packages + 1, 0);
  // slot[u]: the local index of level-unique u in the package being built (stamp = the
  // package + 1, so nothing is cleared between packages).
  std::vector<u16> slot(uniques, 0);
  std::vector<u32> stamp(uniques, 0);
  {
    // `src` is made at its exact size: a dry run of the packaging below counts it.
    u32 total = 0;
    for (u32 p = 0; p < out.packages; ++p) {
      const u32 lo = p * packageSize;
      const u32 n = out.packageCorners(p);
      for (u32 k = 0; k < n; ++k) {
        const u32 u = out.uid[lo + k];
        if (stamp[u] != p + 1) {
          stamp[u] = p + 1;
          ++total;
        }
      }
    }
    // the package-uniques' list, at the size the dry run counted
    if (!room(vectorBytes(total, sizeof(u32)))) {
      out = Level();
      if (refused != nullptr) *refused = true;
      return false;
    }
    out.src.reserve(total);
    std::fill(stamp.begin(), stamp.end(), 0u);
  }
  for (u32 p = 0; p < out.packages; ++p) {
    const u32 lo = p * packageSize;
    const u32 n = out.packageCorners(p);
    u32 count = 0;
    for (u32 k = 0; k < n; ++k) {
      const u32 u = out.uid[lo + k];
      if (stamp[u] != p + 1) {
        stamp[u] = p + 1;
        slot[u] = (u16)count++;
        out.src.push_back(u);
      }
      out.local[lo + k] = slot[u];
    }
    out.first[p + 1] = out.first[p] + count;
    if (!fits(n, count)) return false;
  }
  return true;
}

void orderTriangles(const u32* skinSource, u32 corners, u32 packageSize,
                    std::vector<u32>& order) {
  const u32 tris = corners / 3;
  order.clear();
  if (tris == 0 || packageSize < 3) return;
  order.reserve(tris);  // exact: the caller asked for this much before calling
  const u32 perPackage = packageSize / 3;
  // Level-unique id of each corner, and the triangles of each unique.
  std::vector<u32> uid(corners);
  u32 uniques = 0;
  for (u32 v = 0; v < corners; ++v)
    uid[v] = skinSource[v] == v ? uniques++ : uid[skinSource[v]];
  std::vector<u32> off(uniques + 1, 0);
  for (u32 v = 0; v < tris * 3; ++v) ++off[uid[v] + 1];
  for (u32 u = 0; u < uniques; ++u) off[u + 1] += off[u];
  std::vector<u32> adj(off[uniques]);
  std::vector<u32> fill(off.begin(), off.end() - 1);
  for (u32 v = 0; v < tris * 3; ++v) adj[fill[uid[v]]++] = v / 3;
  std::vector<u8> used(tris, 0);
  std::vector<u32> inPkg(uniques, 0);  // package number + 1 when the unique is in the open package
  std::vector<u32> cand;
  u32 nextSeed = 0, pkg = 0;
  while (order.size() < tris) {
    ++pkg;
    cand.clear();
    for (u32 placed = 0; placed < perPackage && order.size() < tris; ++placed) {
      // Candidates: unused triangles touching the open package; else a new seed.
      u32 best = 0xFFFFFFFFu, bestCost = 4;
      for (size_t i = 0; i < cand.size();) {
        const u32 t = cand[i];
        if (used[t]) {
          cand[i] = cand.back();
          cand.pop_back();
          continue;
        }
        u32 cost = 0;
        for (int c = 0; c < 3; ++c)
          if (inPkg[uid[3 * t + c]] != pkg) ++cost;
        if (cost < bestCost || (cost == bestCost && t < best)) {
          bestCost = cost;
          best = t;
        }
        ++i;
      }
      if (best == 0xFFFFFFFFu) {
        while (used[nextSeed]) ++nextSeed;
        best = nextSeed;
      }
      used[best] = 1;
      order.push_back(best);
      for (int c = 0; c < 3; ++c) {
        const u32 u = uid[3 * best + c];
        if (inPkg[u] == pkg) continue;
        inPkg[u] = pkg;
        for (u32 k = off[u]; k < off[u + 1]; ++k)
          if (!used[adj[k]]) cand.push_back(adj[k]);
      }
    }
  }
}

void layoutPose(const Level& level, PoseLayout& out) {
  out.block.assign(level.packages + 1, 0);
  for (u32 p = 0; p < level.packages; ++p)
    out.block[p + 1] = out.block[p] + PoseLayout::blockBytes(level.packageUniques(p));
}

void initPose(const Level& level, const PoseLayout& layout, u8* buf) {
  std::memset(buf, 0, layout.bytes());
  for (u32 p = 0; p < level.packages; ++p) {
    const u32 u = level.packageUniques(p);
    const u32 base = uniqueAddr(level.packageCorners(p));
    u8* b = buf + layout.block[p];
    putWords(b, kStmask, kPosMask, kStmod | 1u,
             unpack(Compact::Format::V4_16, true, u, base));
    putWords(b + PoseLayout::nrmHeader(u), kNop, kNop, kStmod,
             unpack(Compact::Format::V4_8, false, u, base + kUniqueSplit));
  }
}

void writePose(const Level& level, const PoseLayout& layout, const s16* qPos, const s8* qNrm,
               u8* buf) {
  for (u32 p = 0; p < level.packages; ++p) {
    const u32 u = level.packageUniques(p);
    u8* b = buf + layout.block[p];
    u8* pos = b + 16;
    u8* nrm = b + PoseLayout::nrmHeader(u) + 16;
    const u32* src = level.src.data() + level.first[p];
    for (u32 j = 0; j < u; ++j) {
      std::memcpy(pos + j * kPosBytes, qPos + (size_t)src[j] * 4, kPosBytes);
      std::memcpy(nrm + j * kNrmBytes, qNrm + (size_t)src[j] * 4, kNrmBytes);
    }
  }
}

void layoutStatic(const Level& level, StaticLayout& out) {
  out.block.assign(level.packages + 1, 0);
  for (u32 p = 0; p < level.packages; ++p)
    out.block[p + 1] = out.block[p] + StaticLayout::blockBytes(level.packageCorners(p));
}

void buildStatic(const Level& level, const StaticLayout& layout, const s16* stq,
                 const u32* entry, const u32* kickCodes, u8* out) {
  std::memset(out, 0, layout.bytes());
  for (u32 p = 0; p < level.packages; ++p) {
    const u32 n = level.packageCorners(p);
    const u32 lo = p * level.packageSize;
    u8* b = out + layout.block[p];
    if (entry != nullptr) std::memcpy(b, entry + 12 * (size_t)p, StaticLayout::kEntryBytes);
    u8* hdr = b + StaticLayout::kEntryBytes;
    putWords(hdr, kNop, kNop, kStmod, unpack(Compact::Format::V4_16, false, n, kCornerAddr));
    u8* table = hdr + 16;
    const s16 base = (s16)uniqueAddr(n);
    for (u32 k = 0; k < n; ++k) {
      s16 e[4] = {stq[3 * (size_t)(lo + k) + 0], stq[3 * (size_t)(lo + k) + 1],
                  stq[3 * (size_t)(lo + k) + 2], (s16)(base + (s16)level.local[lo + k])};
      std::memcpy(table + k * kCornerBytes, e, kCornerBytes);
    }
    u8* kick = table + StaticLayout::cornerQw(n) * 16;
    putWords(kick, kStmod, kNop, kickCodes ? kickCodes[0] : kNop,
             kickCodes ? kickCodes[1] : kNop);
  }
}

Stats statsOf(const Level& level, const StaPipCompactLayout* plainLayout) {
  Stats s;
  s.corners = level.corners;
  s.uniques = level.uniques;
  s.packageUniques = level.totalUniques();
  s.packages = level.packages;
  s.minPackageUniques = 0xFFFFFFFFu;
  PoseLayout pl;
  layoutPose(level, pl);
  StaticLayout sl;
  layoutStatic(level, sl);
  s.poseBytes = pl.bytes();
  s.staticBytes = sl.bytes();
  s.gatherBytes = s.packageUniques * 2;
  for (u32 p = 0; p < level.packages; ++p) {
    const u32 u = level.packageUniques(p);
    if (u > s.maxPackageUniques) s.maxPackageUniques = u;
    if (u < s.minPackageUniques) s.minPackageUniques = u;
    if (plainLayout != nullptr) {
      const u32 n = level.packageCorners(p);
      u32 qw = 3 + 1;  // entry, kick
      for (u32 a = 0; a < plainLayout->attrCount; ++a)
        qw += 1 + Compact::runQwords(plainLayout->attrs[a], n);
      s.plainBytes += qw * 16;
    }
  }
  return s;
}

}  // namespace Indexed
}  // namespace SkelSkin
}  // namespace Tyra
