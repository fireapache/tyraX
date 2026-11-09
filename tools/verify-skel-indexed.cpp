// Host verifier for the skinned stream with indexed corners
// (renderer/3d/mesh/dynamic/skel_indexed.hpp; docs/animated-models.md, "Skinned streams with
// indexed corners").
//
// For every part and LOD of the real soldier model and each stream package size tried:
//  (a) packaging: the packages cover every corner exactly once, every local index is in range
//      and names the corner's own unique, every package fits one half of the VU1 double buffer
//      (the buffer layout is printed), and the shared gather list names only level uniques;
//  (b) the byte-exact decode: over 2000 poses, the shipped unique-vertex path
//      (Unique::blend + SkelSkin::writeUniqueStream) writes the plain corner stream; the same
//      blend is quantized once, gathered into the pose blocks (writePose), and the pose and
//      static blocks are put through a model of the VIF (the three UNPACKs the layout uses,
//      STMASK, STMOD, ROW) and of the VU1 corner fetch (`ilw.w` of the corner's address, the
//      position at it, the normal kUniqueSplit quadwords on). The position, ST and normal
//      runs of every package that come out are compared BYTE FOR BYTE (padding included) with
//      the plain stream's, and so are the part boxes (partBox) formed from the decoded corners;
//  (c) the numbers: uniques per package, the duplication factor, stores per unique, the pose
//      buffer, the static blocks and the plain stream in bytes per part level and per fighter.
//  (d) the VU1 arm: for sample poses, every package is staged twice in VU1 data memory through
//      the VIF decoder of the capture tooling (vucap) - the plain package's three runs for the
//      shipped cull TD program, and the pose block + static block for the indexed program - and
//      both programs run in the VU simulator: every GS word of every package must be identical
//      (the ST register's ignored fourth lane aside);
//  The batch kernel arm: the lane model of the MMI batch kernel (skel_batch.hpp) writes the
//  same level-unique arrays; the pose gathered from them decodes to the scatter's corners.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel does this):
//   build/verify-skel-indexed <model>.tskl

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_batch.hpp"
#include "renderer/3d/mesh/dynamic/skel_indexed.hpp"
#include "renderer/3d/mesh/dynamic/skel_indexed_shared.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/3d/mesh/dynamic/skel_unique.hpp"
#include "../src/vucap.hpp"
#include "../src/vugen.hpp"
#include "../src/vusim.hpp"

namespace {

using namespace Tyra;
using SkelSkin::SkinScatter;
namespace Ix = SkelSkin::Indexed;

int checks = 0;
int failures = 0;

void check(bool ok, const char* what) {
  ++checks;
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

// A column-major palette slot: rigid rotation, scale near 1, translation up to maxShift.
void randomSlot(std::mt19937& rng, float* m, float maxShift, float scaleSpread) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  double qv[4] = {gauss(rng), gauss(rng), gauss(rng), gauss(rng)};
  const double len =
      std::sqrt(qv[0] * qv[0] + qv[1] * qv[1] + qv[2] * qv[2] + qv[3] * qv[3]);
  for (double& c : qv) c /= len;
  const double x = qv[0], y = qv[1], z = qv[2], w = qv[3];
  const double r[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
      {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
      {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  const double s = 1.0 + scaleSpread * uni(rng);
  for (int c = 0; c < 3; ++c)
    for (int row = 0; row < 3; ++row) m[c * 4 + row] = (float)(s * r[row][c]);
  for (int row = 0; row < 3; ++row) m[12 + row] = (float)(maxShift * uni(rng));
  m[3] = m[7] = m[11] = 0.0F;
  m[15] = 1.0F;
}

// The first identical corner of each corner (as repackBind keys them: exact bind bytes).
std::vector<u32> firstIdentical(const SkelMesh& mesh) {
  std::vector<u32> src(mesh.vertexCount);
  std::map<std::array<int, 15>, u32> first;
  for (u32 v = 0; v < mesh.vertexCount; ++v) {
    std::array<int, 15> key{};
    for (int i = 0; i < 3; ++i) key[i] = mesh.pos[3 * v + i];
    for (int i = 0; i < 4; ++i) key[3 + i] = mesh.nrm[4 * v + i];
    for (int i = 0; i < 4; ++i) key[7 + i] = mesh.joints[4 * v + i];
    for (int i = 0; i < 4; ++i) key[11 + i] = mesh.weights[4 * v + i];
    const auto it = first.emplace(key, v);
    src[v] = it.first->second;
  }
  return src;
}

// ---------------------------------------------------------------------------------------
// A model of the VIF as the indexed layout uses it, and of the VU1 corner fetch.

struct Vif {
  u32 mask = 0;
  u32 mode = 0;
  u32 row[4] = {0, 0, 0, 0};
};

// Runs the VIFcodes of `data` (a pose block or a static block, whole quadwords): NOP, STMASK,
// STMOD and UNPACK of V4-16 / V4-8 (signed, TOPS-relative, STCYCL 1/1). `image` is one half of
// the double buffer, TOPS = 0: kHalfQw quadwords of four words.
bool vifRun(const u8* data, u32 bytes, Vif& vif, std::vector<u32>& image, const char** err) {
  std::vector<u32> w(bytes / 4);
  std::memcpy(w.data(), data, bytes);
  size_t i = 0;
  while (i < w.size()) {
    const u32 code = w[i++];
    if (code == 0) continue;
    const u32 cmd = code >> 24;
    if (cmd == 0x20) {  // STMASK
      if (i >= w.size()) return *err = "STMASK without its word", false;
      vif.mask = w[i++];
    } else if (cmd == 0x05) {  // STMOD
      vif.mode = code & 3u;
    } else if ((cmd & 0x60) == 0x60) {  // UNPACK
      const u32 fmt = cmd & 0xFu;
      const bool masked = (cmd & 0x10u) != 0;
      u32 num = (code >> 16) & 0xFFu;
      if (num == 0) num = 256;
      const u32 addr = code & 0x3FFu;
      if ((code & 0x8000u) == 0) return *err = "an UNPACK that is not TOPS-relative", false;
      if (code & 0x4000u) return *err = "an UNPACK with USN set", false;
      if (fmt != 0xD && fmt != 0xE) return *err = "an UNPACK format the layout does not use", false;
      const u32 size = fmt == 0xD ? 8 : 4;
      const u32 payloadWords = (num * size + 3) / 4;
      if (i + payloadWords > w.size()) return *err = "an UNPACK past its block", false;
      if (addr + num > Ix::kHalfQw) return *err = "an UNPACK past the buffer half", false;
      const u8* p = reinterpret_cast<const u8*>(&w[i]);
      for (u32 e = 0; e < num; ++e) {
        for (u32 f = 0; f < 4; ++f) {
          u32 lane;
          if (fmt == 0xD) {
            s16 h;
            std::memcpy(&h, p + e * 8 + f * 2, 2);
            lane = (u32)(s32)h;
          } else {
            lane = (u32)(s32)(s8)p[e * 4 + f];
          }
          const u32 m = masked ? (vif.mask >> (f * 2)) & 3u : 0u;
          if (m == 0) {
            if (vif.mode == 1) lane += vif.row[f];
          } else if (m == 2) {
            lane = 0x3F800000u;  // COL
          } else {
            return *err = "a mask the layout does not use", false;
          }
          image[(size_t)(addr + e) * 4 + f] = lane;
        }
      }
      i += payloadWords;
    } else if (cmd == 0x11 || cmd == 0x14) {  // FLUSH, MSCAL: no data
    } else {
      return *err = "an unexpected VIFcode", false;
    }
  }
  return true;
}

// One package's corners as VU1 reads them from `image`: per corner the address word
// (`ilw.w`), the position lanes at it and the normal lanes kUniqueSplit quadwords on.
struct Corner {
  s32 pos[3];
  s32 nrm[3];
  s32 stq[3];
  u32 posW;
};

bool fetchCorner(const std::vector<u32>& image, u32 k, Corner& c) {
  const u32* e = &image[(size_t)(Ix::kCornerAddr + k) * 4];
  const u32 addr = e[3] & 0xFFFFu;
  if (addr + Ix::kUniqueSplit >= Ix::kHalfQw) return false;
  const u32* pos = &image[(size_t)addr * 4];
  const u32* nrm = &image[(size_t)(addr + Ix::kUniqueSplit) * 4];
  for (int a = 0; a < 3; ++a) {
    c.pos[a] = (s32)pos[a];
    c.nrm[a] = (s32)nrm[a];
    c.stq[a] = (s32)e[a];
  }
  c.posW = pos[3];
  return true;
}

// ---------------------------------------------------------------------------------------

struct Totals {
  u64 corners = 0, uniques = 0, packageUniques = 0, packages = 0;
  u64 poseBytes = 0, staticBytes = 0, gatherBytes = 0, plainBytes = 0;
  u64 maxU = 0, minU = 1u << 30;
};

struct LevelData {
  const SkelMesh* mesh = nullptr;
  std::vector<u32> src;
  SkelSkin::Unique::UniqueBind ub;
  char label[48];
  size_t part = 0, lod = 0;
};

// Everything one (level, package size) pair needs.
struct Case {
  LevelData* lv = nullptr;
  u32 size = 0;
  bool sorted = false;  // the part's triangles reordered for locality (orderTriangles)
  SkelMesh perm;        // the reordered copy when sorted
  const SkelMesh* mesh = nullptr;
  std::vector<u32> src;
  SkelSkin::Unique::UniqueBind ub;
  Ix::Level level;
  Ix::PoseLayout poseLayout;
  Ix::StaticLayout staticLayout;
  std::vector<u8> pose, stat;
  std::vector<s16> stq;  // the plain stream's ST run elements, [corners][3]
  SkinScatter scatter;
  StaPipCompactStream stream;
  StaPipCompactLayout layout;
  s32 stSlot = -1, nrmSlot = -1;
  // The engine's side (Shared, an indexed StaPipCompactStream): built on the first pose.
  std::unique_ptr<Ix::Shared> shared;
  Ix::PoseBuf* buf = nullptr;
  StaPipCompactStream istream;
};

// A copy of `m` with its triangles in `order` (position, normal, uv, joints, weights a corner).
SkelMesh permuted(const SkelMesh& m, const std::vector<u32>& order) {
  SkelMesh r = m;
  for (size_t i = 0; i < order.size(); ++i)
    for (int c = 0; c < 3; ++c) {
      const size_t from = 3 * (size_t)order[i] + c, to = 3 * i + c;
      std::memcpy(&r.pos[3 * to], &m.pos[3 * from], 6);
      std::memcpy(&r.nrm[4 * to], &m.nrm[4 * from], 4);
      if (!m.uv.empty()) std::memcpy(&r.uv[2 * to], &m.uv[2 * from], 4);
      std::memcpy(&r.joints[4 * to], &m.joints[4 * from], 4);
      std::memcpy(&r.weights[4 * to], &m.weights[4 * from], 4);
    }
  return r;
}

bool setUp(Case& c, const Compact::Box& modelBox, const Compact::Box& outBox,
           const Compact::Box& bind, const SkelModel& model) {
  c.mesh = c.lv->mesh;
  if (c.sorted) {
    std::vector<u32> order;
    Ix::orderTriangles(c.lv->src.data(), c.lv->mesh->vertexCount, c.size, order);
    c.perm = permuted(*c.lv->mesh, order);
    c.mesh = &c.perm;
  }
  const SkelMesh& mesh = *c.mesh;
  c.src = firstIdentical(mesh);
  SkelSkin::Unique::buildUnique(mesh.pos.data(), mesh.nrm.data(), mesh.joints.data(),
                                mesh.weights.data(), c.src.data(), mesh.vertexCount, bind, c.ub);
  const u32 n = mesh.vertexCount;
  const bool textured = !mesh.uv.empty();
  c.layout = StaPipCompactLayout::forClass(textured, false, true, false, c.size, false);
  c.stSlot = c.layout.indexOf(Compact::Attr::St);
  c.nrmSlot = c.layout.indexOf(Compact::Attr::Normal);
  std::vector<float> tp(3 * (size_t)n), tn(3 * (size_t)n), tuv(textured ? 2 * (size_t)n : 0);
  decodeMesh(modelBox, mesh, tp.data(), tn.data(), textured ? tuv.data() : nullptr);
  std::vector<float> spos(4 * (size_t)n), snrm(4 * (size_t)n), sst(4 * (size_t)n, 0.0F);
  for (u32 v = 0; v < n; ++v) {
    for (int i = 0; i < 3; ++i) {
      spos[4 * v + i] = tp[3 * v + i];
      snrm[4 * v + i] = tn[3 * v + i];
    }
    spos[4 * v + 3] = 1.0F;
    sst[4 * v + 0] = textured ? tuv[2 * v] : 0.0F;
    sst[4 * v + 1] = textured ? tuv[2 * v + 1] : 0.0F;
    sst[4 * v + 2] = 1.0F;
  }
  const StaPipCompactSource source{spos.data(), sst.data(), nullptr, snrm.data(), n};
  if (!Compact::packArrays(c.layout, source, outBox, c.stream)) return false;
  (void)model;
  // The plain stream's ST run elements, package by package (V3-16: six bytes a corner).
  c.stq.assign(3 * (size_t)n, 0);
  if (c.stSlot >= 0)
    for (u32 v = 0; v < n; ++v) {
      const u32 p = v / c.size, k = v % c.size;
      std::memcpy(&c.stq[3 * (size_t)v], c.stream.run(p, (u8)c.stSlot) + k * 6, 6);
    }
  if (!Ix::build(c.src.data(), n, c.size, c.level)) return false;
  Ix::layoutPose(c.level, c.poseLayout);
  Ix::layoutStatic(c.level, c.staticLayout);
  c.pose.assign(c.poseLayout.bytes(), 0);
  c.stat.assign(c.staticLayout.bytes(), 0);
  Ix::initPose(c.level, c.poseLayout, c.pose.data());
  Ix::buildStatic(c.level, c.staticLayout, c.stq.data(), nullptr, nullptr, c.stat.data());
  SkelSkin::buildScatter(c.src.data(), n, c.size, c.scatter);
  return true;
}

// (a) The packaging.
void checkPackaging(const Case& c) {
  const Ix::Level& L = c.level;
  const u32 n = L.corners;
  bool covered = L.first.size() == L.packages + 1 && L.first[0] == 0;
  bool inRange = true, ownUnique = true, fitsAll = true, srcOk = true, sizesOk = true;
  u32 counted = 0;
  for (u32 p = 0; p < L.packages; ++p) {
    const u32 pc = L.packageCorners(p);
    counted += pc;
    if (pc == 0 || (p + 1 < L.packages && pc != L.packageSize)) sizesOk = false;
    const u32 u = L.packageUniques(p);
    if (!Ix::fits(pc, u)) fitsAll = false;
    for (u32 k = 0; k < pc; ++k) {
      const u32 v = p * L.packageSize + k;
      if (L.local[v] >= u) {
        inRange = false;
        continue;
      }
      if (L.src[L.first[p] + L.local[v]] != L.uid[v]) ownUnique = false;
    }
    for (u32 j = 0; j < u; ++j)
      if (L.src[L.first[p] + j] >= L.uniques) srcOk = false;
  }
  check(covered && counted == n && sizesOk, "the packages cover every corner exactly once");
  check(inRange, "every local index is inside its package's unique list");
  check(ownUnique, "every corner's local index names the corner's own unique");
  check(srcOk, "the gather list names only level uniques");
  check(fitsAll, "every package fits one half of the VU1 double buffer");
}

// Decodes package p from the pose and static blocks into the plain stream's runs.
bool decodePackage(const Case& c, u32 p, const u32 row[4], std::vector<u32>& image,
                   std::vector<u8>& posRun, std::vector<u8>& nrmRun, std::vector<u8>& stRun,
                   std::vector<s32>& lo, std::vector<s32>& hi, std::vector<s32>& cornerPos) {
  const Ix::Level& L = c.level;
  const u32 pc = L.packageCorners(p);
  std::fill(image.begin(), image.end(), 0xDEADBEEFu);
  Vif vif;
  for (int a = 0; a < 4; ++a) vif.row[a] = row[a];
  const char* err = nullptr;
  const u32 pb = c.poseLayout.block[p], pe = c.poseLayout.block[p + 1];
  const u32 sb = c.staticLayout.block[p], se = c.staticLayout.block[p + 1];
  if (!vifRun(c.pose.data() + pb, pe - pb, vif, image, &err) ||
      !vifRun(c.stat.data() + sb, se - sb, vif, image, &err)) {
    std::printf("FAIL: VIF model: %s\n", err);
    ++failures;
    return false;
  }
  posRun.assign((size_t)Compact::runQwords(Compact::Attr::Position, pc) * 16, 0);
  nrmRun.assign((size_t)Compact::runQwords(Compact::Attr::Normal, pc) * 16, 0);
  stRun.assign((size_t)Compact::runQwords(Compact::Attr::St, pc) * 16, 0);
  for (u32 k = 0; k < pc; ++k) {
    Corner cr;
    if (!fetchCorner(image, k, cr)) return false;
    if (cr.posW != 0x3F800000u) return false;  // w must be COL's 1.0
    for (int a = 0; a < 3; ++a) {
      const s32 q = cr.pos[a] - (s32)row[a];
      if (q < -32768 || q > 32767) return false;
      const s16 h = (s16)q;
      std::memcpy(&posRun[(size_t)k * 6 + a * 2], &h, 2);
      cornerPos[3 * (size_t)k + a] = q;
      const s8 b = (s8)cr.nrm[a];
      nrmRun[(size_t)k * 4 + a] = (u8)b;
      const s16 st = (s16)cr.stq[a];
      std::memcpy(&stRun[(size_t)k * 6 + a * 2], &st, 2);
    }
  }
  (void)lo;
  (void)hi;
  return true;
}

// ---------------------------------------------------------------------------------------
// The VU1 arm.

struct VuArm {
  vugen::Desc compactDesc, indexedDesc;
  vuir::Program compactProg, indexedProg;
  std::vector<uint32_t> base;  // constants + header from vugen's staging
  int top = 22;
  vugen::Tolerance tol;
  u64 packages = 0, controlsCaught = 0, controls = 0;
  bool reported = false;
};

// Lands the unpacks `words` perform in `mem` (TOPS-relative, TOPS = top).
bool landUnpacks(const std::vector<uint32_t>& words, std::vector<uint32_t>& mem, int top) {
  vucap::VifRegs regs;
  std::string err;
  const std::vector<vucap::Unpack> unpacks = vucap::decodeVif(words, regs, &err);
  if (!err.empty()) return false;
  for (const vucap::Unpack& u : unpacks) {
    if (!u.captured || !u.useTops) return false;
    for (size_t i = 0; i < u.words.size(); ++i)
      if (!u.protect[i]) mem[(size_t)(top + (int)u.vuAddr) * 4 + i] = u.words[i];
  }
  return true;
}

std::vector<uint32_t> bytesToWords(const u8* p, size_t bytes) {
  std::vector<uint32_t> w(bytes / 4);
  std::memcpy(w.data(), p, bytes);
  return w;
}

void vuPackage(VuArm& vu, Case& c, u32 p, const Compact::Box& box, const char* what) {
  const Ix::Level& L = c.level;
  const u32 n = L.packageCorners(p);
  // The bag setup both sides start from (STCYCL 1/1, ROW = the box's origin index, COL).
  u32 setup[Compact::kBagSetupWords];
  Compact::bagSetup(box, setup);
  std::vector<uint32_t> plain(setup, setup + Compact::kBagSetupWords);
  for (u8 a = 0; a < c.layout.attrCount; ++a) {
    const u32* h = reinterpret_cast<const u32*>(c.stream.runHeader(p, a));
    plain.insert(plain.end(), h, h + 4);
    const std::vector<uint32_t> run =
        bytesToWords(c.stream.run(p, a), (size_t)c.stream.runQwords(p, a) * 16);
    plain.insert(plain.end(), run.begin(), run.end());
  }
  std::vector<uint32_t> indexed(setup, setup + Compact::kBagSetupWords);
  {
    const std::vector<uint32_t> pb = bytesToWords(c.pose.data() + c.poseLayout.block[p],
                                                   c.poseLayout.block[p + 1] - c.poseLayout.block[p]);
    const std::vector<uint32_t> sb = bytesToWords(c.stat.data() + c.staticLayout.block[p],
                                                   c.staticLayout.block[p + 1] - c.staticLayout.block[p]);
    indexed.insert(indexed.end(), pb.begin(), pb.end());
    indexed.insert(indexed.end(), sb.begin(), sb.end());
  }
  std::vector<uint32_t> memE = vu.base, memI = vu.base;
  // The buffer header both programs read: the count in the low ten bits of w.
  for (std::vector<uint32_t>* m : {&memE, &memI}) {
    uint32_t& w = (*m)[(size_t)vu.top * 4 + 3];
    w = (w & ~0x3FFu) | n;
    // ... and the prim tag's NLOOP: one GS vertex a corner.
    uint32_t& tag = (*m)[(size_t)(vu.top + 1) * 4 + 0];
    tag = (tag & ~0x7FFFu) | n;
  }
  if (!landUnpacks(plain, memE, vu.top) || !landUnpacks(indexed, memI, vu.top)) {
    check(false, "the VIF decoder lands both package streams");
    return;
  }
  char ctx[96];
  std::snprintf(ctx, sizeof ctx, "%s package %u of %u", what, p, L.packages);
  ++vu.packages;
  if (!vugen::compareIndexedDraws(vu.compactDesc, vu.compactProg, memE, vu.indexedProg, memI,
                                  vu.top, ctx, vu.tol)) {
    if (!vu.reported) {  // the first one only: the tolerance stays failed after it
      vu.reported = true;
      std::printf("FAIL: VU1 arm: %s: %s %s\n", ctx, vu.tol.error.c_str(), vu.tol.detail.c_str());
    }
  }
  // The control: one unique position moved by 4 steps must change the picture.
  if (p % 7 == 0) {
    std::vector<uint32_t> memC = memI;
    const int at = vu.top + (int)Ix::uniqueAddr(n);
    memC[(size_t)at * 4 + 0] += 64;
    vugen::Tolerance scratch;
    scratch.within = true;
    ++vu.controls;
    if (!vugen::compareIndexedDraws(vu.compactDesc, vu.compactProg, memE, vu.indexedProg, memC,
                                    vu.top, ctx, scratch))
      ++vu.controlsCaught;
  }
}

struct PoseCheck {
  u64 packages = 0, packageBad = 0, boxBad = 0, poses = 0;
  u64 streamCorners = 0, streamBad = 0, copyBad = 0;
};

// (b2) The engine's indexed stream (Shared + StaPipCompactStream::allocateIndexed) against the plain
// stream the shipped path wrote for the same pose: every corner's position, normal and ST through
// the view, and the copy runs the expansion for the clip routes reads (copyRun), element for
// element. The shared tables (ranges, gather) are checked once per case.
void checkIndexedStream(Case& c, const s16* qPos, const s8* qNrm, const Compact::Box& box,
                        PoseCheck& out) {
  const u32 n = c.mesh->vertexCount;
  if (!c.shared) {
    c.shared.reset(new Ix::Shared());
    const bool built = c.shared->build(c.src.data(), n, c.size, c.stq.data(), c.layout);
    check(built, "the engine's shared packaging builds");
    if (!built) return;
    c.buf = c.shared->acquire();
    check(c.buf != nullptr, "a pose buffer is acquired");
    if (c.buf == nullptr) return;
    check(c.istream.allocateIndexed(c.layout, n, box, &c.shared->view(), c.buf->data),
          "an indexed stream allocates");
    // The ranges: unique u falls in the parts of its corners, exactly the scatter's list.
    bool rangesOk = c.shared->level().uniques == c.ub.count;
    for (u32 u = 0; rangesOk && u < c.ub.count; ++u) {
      const u32 src = c.ub.corner[u];
      const u32 a = c.shared->rangeOffsets()[u], b = c.shared->rangeOffsets()[u + 1];
      if (b - a != c.scatter.rngOff[src + 1] - c.scatter.rngOff[src]) rangesOk = false;
      for (u32 k = 0; rangesOk && k < b - a; ++k)
        if (c.shared->ranges()[a + k] != c.scatter.rng[c.scatter.rngOff[src] + k]) rangesOk = false;
    }
    check(rangesOk, "the shared range lists are the scatter's");
    check(c.shared->partCount() == c.stream.partCount(), "the shared part count is the stream's");
  }
  if (c.buf == nullptr || !c.istream.allocated()) return;
  c.istream.rebox(box);
  Ix::writePose(c.shared->level(), c.shared->pose(), qPos, qNrm, c.buf->data);
  StaPipCompactView plain(c.stream), idx(c.istream);
  bool bad = false;
  for (u32 v = 0; v < n; ++v) {
    float a[4], b[4], na[3], nb[3], sa[3], sb[3];
    plain.position(v, a);
    idx.position(v, b);
    plain.normal(v, na);
    idx.normal(v, nb);
    plain.stq(v, sa);
    idx.stq(v, sb);
    for (int i = 0; i < 3; ++i)
      if (a[i] != b[i] || na[i] != nb[i] || sa[i] != sb[i]) bad = true;
  }
  out.streamCorners += n;
  if (bad) ++out.streamBad;
  // copyRun: three runs over a package-sized stretch from an arbitrary start, and scattered.
  std::vector<u8> ra, rb;
  const u32 first = (u32)(out.poses * 37 % (n > c.size ? n - c.size : 1));
  const u32 count = std::min<u32>(c.size, n - first);
  for (u8 a = 0; a < c.layout.attrCount; ++a) {
    const u32 qw = Compact::runQwords(c.layout.attrs[a], count) * 16;
    ra.assign(qw, 0xAB);
    rb.assign(qw, 0xCD);
    Compact::copyRun(c.stream, a, nullptr, first, count, ra.data());
    Compact::copyRun(c.istream, a, nullptr, first, count, rb.data());
    if (ra != rb) ++out.copyBad;
  }
}

// (b) One pose of one case: compare the decode with the plain stream the shipped path wrote.
void checkPose(Case& c, const float* posU, const float* nrmU, PoseCheck& out, bool& clampedAny) {
  const SkelMesh& mesh = *c.mesh;
  const u32 n = mesh.vertexCount;
  const SkelSkin::SkelSkinPart part{mesh.pos.data(), mesh.nrm.data(), mesh.joints.data(),
                                    mesh.weights.data(), c.src.data(), n};
  SkelSkin::SkelSkinPart withMap = part;
  withMap.scatter = &c.scatter;
  const u32 plainClamped =
      SkelSkin::writeUniqueStream(posU, nrmU, c.ub.corner.data(), c.ub.count, withMap, c.stream);
  if (plainClamped == SkelSkin::kSkinRefused) {
    check(false, "the plain unique stream write was refused");
    return;
  }
  if (plainClamped) clampedAny = true;
  // The indexed side: the same box, the same quantize, once per level unique; then the gather.
  const Compact::Box box = c.stream.box();
  s32 oi[3];
  SkelSkin::Unique::poseOrigin(posU, c.ub.count, box.step, oi);
  std::vector<s16> qPos((size_t)c.ub.count * 4);
  std::vector<s8> qNrm((size_t)c.ub.count * 4);
  std::vector<u8> clamp(c.ub.count);
  SkelSkin::Unique::quantize(posU, nrmU, c.ub.count, box.step, oi, qPos.data(), qNrm.data(),
                             clamp.data());
  Ix::writePose(c.level, c.poseLayout, qPos.data(), qNrm.data(), c.pose.data());
  checkIndexedStream(c, qPos.data(), qNrm.data(), box, out);
  u32 row[4];
  Compact::rowFor(box, row);
  static std::vector<u32> image(Ix::kHalfQw * 4);
  std::vector<u8> posRun, nrmRun, stRun;
  const u32 parts = c.stream.partCount();
  const u32 partSize = c.size / 3;
  std::vector<s32> lo(3 * (size_t)parts, 32767), hi(3 * (size_t)parts, -32768), cornerPos;
  for (u32 p = 0; p < c.level.packages; ++p) {
    ++out.packages;
    const u32 pc = c.level.packageCorners(p);
    cornerPos.assign(3 * (size_t)pc, 0);
    bool ok = decodePackage(c, p, row, image, posRun, nrmRun, stRun, lo, hi, cornerPos);
    if (ok) {
      const u8* pp = c.stream.run(p, 0);
      const u8* nn = c.stream.run(p, (u8)c.nrmSlot);
      ok = std::memcmp(pp, posRun.data(), posRun.size()) == 0 &&
           std::memcmp(nn, nrmRun.data(), nrmRun.size()) == 0;
      if (ok && c.stSlot >= 0)
        ok = std::memcmp(c.stream.run(p, (u8)c.stSlot), stRun.data(), stRun.size()) == 0;
      // The padding is part of the run: it must be the plain stream's zero.
      if (ok)
        for (size_t i = (size_t)pc * 6; i < posRun.size(); ++i)
          if (posRun[i] != 0) ok = false;
    }
    if (!ok) ++out.packageBad;
    for (u32 k = 0; ok && k < pc; ++k) {
      const u32 part = (p * c.size + k) / partSize;
      for (int a = 0; a < 3; ++a) {
        const s32 q = cornerPos[3 * (size_t)k + a];
        if (q < lo[3 * (size_t)part + a]) lo[3 * (size_t)part + a] = q;
        if (q > hi[3 * (size_t)part + a]) hi[3 * (size_t)part + a] = q;
      }
    }
  }
  // The part boxes the classifier reads: the plain stream's (setPartBounds) against the ones
  // formed from the decoded corners the same way (step x (row + stored index)).
  bool boxOk = true;
  for (u32 r = 0; r < parts; ++r) {
    float mn[3], mx[3];
    c.stream.partBox(r, mn, mx);
    for (int a = 0; a < 3; ++a) {
      if (mn[a] != box.step * (float)(lo[3 * (size_t)r + a] + (s32)row[a]) ||
          mx[a] != box.step * (float)(hi[3 * (size_t)r + a] + (s32)row[a]))
        boxOk = false;
    }
  }
  if (!boxOk) ++out.boxBad;
  ++out.poses;
}

void addTotals(Totals& t, const Case& c) {
  const Ix::Stats s = Ix::statsOf(c.level, &c.layout);
  t.corners += s.corners;
  t.uniques += s.uniques;
  t.packageUniques += s.packageUniques;
  t.packages += s.packages;
  t.poseBytes += s.poseBytes;
  t.staticBytes += s.staticBytes;
  t.gatherBytes += s.gatherBytes;
  t.plainBytes += s.plainBytes;
  t.maxU = std::max<u64>(t.maxU, s.maxPackageUniques);
  t.minU = std::min<u64>(t.minU, s.minPackageUniques);
}

void printTotals(const char* label, const Totals& t) {
  const double dup = t.uniques ? (double)t.packageUniques / (double)t.uniques : 0.0;
  std::printf(
      "  %-26s corners %5llu  level uniques %4llu  packages %3llu  uniques/package %5.1f "
      "(min %llu max %llu)  duplication %.3f  stores/unique %.3f (plain %.2f)\n"
      "  %-26s bytes: plain stream one side %6llu  pose buffer one side %6llu (%.1f%%)  static "
      "%6llu  gather list %5llu  pose/corner %.2f B (plain %.2f B)\n",
      label, (unsigned long long)t.corners, (unsigned long long)t.uniques,
      (unsigned long long)t.packages,
      t.packages ? (double)t.packageUniques / (double)t.packages : 0.0,
      (unsigned long long)t.minU, (unsigned long long)t.maxU, dup, dup,
      t.uniques ? (double)t.corners / (double)t.uniques : 0.0, "", (unsigned long long)t.plainBytes,
      (unsigned long long)t.poseBytes, t.plainBytes ? 100.0 * t.poseBytes / t.plainBytes : 0.0,
      (unsigned long long)t.staticBytes, (unsigned long long)t.gatherBytes,
      t.corners ? (double)t.poseBytes / (double)t.corners : 0.0,
      t.corners ? (double)t.plainBytes / (double)t.corners : 0.0);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: verify-skel-indexed <model.tskl> [poses]\n");
    return 2;
  }
  const int poseCount = argc > 2 ? std::atoi(argv[2]) : 2000;
  SkelModel model;
  bool loaded = false;
  if (FILE* f = std::fopen(argv[1], "rb")) {
    std::vector<u8> bytes;
    u8 chunk[8192];
    size_t got;
    while ((got = std::fread(chunk, 1, sizeof(chunk), f)) > 0)
      bytes.insert(bytes.end(), chunk, chunk + got);
    std::fclose(f);
    loaded = TsklLoader::parse(bytes.data(), bytes.size(), model);
  }
  check(loaded, "the real model parses");
  if (!loaded) {
    std::printf("verify-skel-indexed: %d checks, %d failed\n", checks, failures);
    return 1;
  }
  const Compact::Box bind = model.box;
  const Compact::Box outBox = skinOutputBox(model);
  const u32 slots = (u32)model.palette.size();
  check(slots <= SkelSkin::kMaxSlots, "the palette fits the skin");

  // The joints each part level uses: what a VU0 batch would have to hold of the palette.
  std::printf("palette: %u slots (4 quadwords each in VU0 data memory = %u qw of 256)\n", slots,
              slots * 4);
  for (size_t pi = 0; pi < model.parts.size(); ++pi) {
    const SkelMesh& m = model.parts[pi];
    SkelSkin::SkelSkinExtent ext;
    SkelSkin::bindExtent(m.pos.data(), m.joints.data(), m.weights.data(), m.vertexCount, ext);
    unsigned usedJoints = 0;
    for (u32 j = 0; j < SkelSkin::kMaxSlots; ++j) usedJoints += (ext.used >> j) & 1ULL;
    std::printf("  part %zu LOD0 uses %u of %u palette slots\n", pi, usedJoints, slots);
  }

  // The VU1 buffer layout, once (n = 48 and 75).
  std::printf("VU1 buffer half: %u qw. Indexed package (n corners, u uniques), TOPS-relative:\n"
              "  qw 0..1 header; qw %u.. corner table (n); output from qw %u: tag block %u + 3n; "
              "uniques at %u + 4n = 11 + 4n: positions (u), normals %u qw on (u)\n",
              Ix::kHalfQw, Ix::kCornerAddr, Ix::kHeaderQw + 0, Ix::kTagBlockQw,
              Ix::kHeaderQw + Ix::kTagBlockQw, Ix::kUniqueSplit);
  for (u32 n : {48u, 75u})
    std::printf("  n=%u: corner table qw 2..%u, output %u..%u, positions %u..%u, normals %u..%u "
                "(u<=n), last qw <= %u of %u\n",
                n, 2 + n - 1, 2 + n, 2 + n + 9 + 3 * n - 1, Ix::uniqueAddr(n),
                Ix::uniqueAddr(n) + n - 1, Ix::uniqueAddr(n) + Ix::kUniqueSplit,
                Ix::uniqueAddr(n) + Ix::kUniqueSplit + n - 1,
                Ix::uniqueAddr(n) + Ix::kUniqueSplit + n - 1, Ix::kHalfQw - 1);
  check(Ix::fits(75, 74) && !Ix::fits(75, 75) && Ix::fits(48, 48) && !Ix::fits(78, 3) &&
            !Ix::fits(75, 76),
        "the buffer fit rule accepts 75 corners with 74 uniques (the renderer's half is 460 qw) "
        "and refuses more");

  std::vector<LevelData> levels;
  levels.reserve(64);
  for (size_t pi = 0; pi < model.parts.size(); ++pi) {
    const size_t nl = 1 + model.parts[pi].lods.size();
    for (size_t l = 0; l < nl && l <= 2; ++l) {
      LevelData lv;
      lv.part = pi;
      lv.lod = l;
      lv.mesh = l == 0 ? static_cast<const SkelMesh*>(&model.parts[pi])
                       : static_cast<const SkelMesh*>(&model.parts[pi].lods[l - 1]);
      lv.src = firstIdentical(*lv.mesh);
      SkelSkin::Unique::buildUnique(lv.mesh->pos.data(), lv.mesh->nrm.data(),
                                    lv.mesh->joints.data(), lv.mesh->weights.data(),
                                    lv.src.data(), lv.mesh->vertexCount, bind, lv.ub);
      std::snprintf(lv.label, sizeof(lv.label), "part %zu LOD%zu", pi, l);
      levels.push_back(std::move(lv));
    }
  }
  check(!levels.empty(), "the model has parts");

  const u32 sizes[] = {24, 36, 48, 60, 75};
  // The cases the byte-exact check runs on: the shipped package (48) and the cull_td maximum
  // (75). The others only feed the numbers table.
  std::vector<Case*> cases;
  std::map<u32, std::vector<Case*>> bySize;
  std::vector<Case*> sortedCases;
  std::map<u32, std::vector<Case*>> sortedBySize;
  for (u32 size : sizes)
    for (LevelData& lv : levels)
      for (int sorted = 0; sorted < 2; ++sorted) {
        Case* c = new Case();
        c->lv = &lv;
        c->size = size;
        c->sorted = sorted != 0;
        if (!setUp(*c, bind, outBox, bind, model)) {
          check(false, "a case builds");
          delete c;
          continue;
        }
        checkPackaging(*c);
        cases.push_back(c);
        (sorted ? sortedBySize : bySize)[size].push_back(c);
      }

  VuArm vu;
  vu.compactDesc = vugen::descCullTextureDirLights();
  vu.indexedDesc = vugen::descCullTextureDirLightsIndexed();
  {
    const vugen::Built bc = vugen::build(vu.compactDesc);
    const vugen::Built bi = vugen::build(vu.indexedDesc);
    check(bc.errors.empty() && bi.errors.empty(), "both TD programs build");
    vu.compactProg = bc.program;
    vu.indexedProg = bi.program;
    const vugen::CompactTrial ct = vugen::stageCompact(vu.compactDesc, 1, 0x51A2u);
    check(ct.error.empty(), "the VU staging builds");
    vu.base = ct.compact;
    vu.top = ct.top;
    vu.tol.ran = vu.tol.within = true;
  }
  check(vugen::kIndexedUniqueSplit == (int)Ix::kUniqueSplit,
        "the program's unique split is the layout's");

  // (b) the byte-exact decode.
  std::mt19937 rng(0x1DEC5u);
  std::vector<float> palette(slots * 16, 0.0F);
  PoseCheck pc;
  bool clampedAny = false;
  std::vector<float> posU, nrmU;
  for (int pose = 0; pose < poseCount; ++pose) {
    if (pose == 0) {
      for (u32 j = 0; j < slots; ++j) {
        float* m = &palette[j * 16];
        std::memset(m, 0, 16 * sizeof(float));
        m[0] = m[5] = m[10] = m[15] = 1.0F;
      }
    } else {
      // Every 100th pose throws the joints 50 m apart so the box clamps (the plain stream
      // and the indexed one must clamp to the same bytes).
      const float shift = pose % 100 == 99 ? 50.0F : 0.3F;
      for (u32 j = 0; j < slots; ++j) randomSlot(rng, &palette[j * 16], shift, 0.1F);
    }
    for (auto* group : {&bySize, &sortedBySize})
      for (u32 size : {48u, 75u})
        for (Case* c : (*group)[size]) {
          posU.assign((size_t)c->ub.count * 4, 0.0F);
          nrmU.assign((size_t)c->ub.count * 4, 0.0F);
          SkelSkin::Unique::blend(c->ub, palette.data(), posU.data(), nrmU.data());
          checkPose(*c, posU.data(), nrmU.data(), pc, clampedAny);
          if (pose % 400 == 1)
            for (u32 p = 0; p < c->level.packages; ++p)
              vuPackage(vu, *c, p, c->stream.box(), c->sorted ? "sorted" : "own order");
        }
  }
  std::printf("byte-exact decode: %llu pose cases (pose x part level x package size x order), %llu packages compared, %llu "
              "packages differ, %llu part-box sets differ\n",
              (unsigned long long)pc.poses, (unsigned long long)pc.packages,
              (unsigned long long)pc.packageBad, (unsigned long long)pc.boxBad);
  check(pc.packages > 0, "packages were compared");
  check(pc.packageBad == 0,
        "the indexed stream decodes BYTE FOR BYTE to the plain corner stream (position, ST, normal)");
  check(pc.boxBad == 0, "the part boxes formed from the decoded corners equal the plain stream's");
  std::printf("engine indexed stream: %llu corners read through the view, %llu poses differ from the "
              "plain stream, %llu copy runs differ\n",
              (unsigned long long)pc.streamCorners, (unsigned long long)pc.streamBad,
              (unsigned long long)pc.copyBad);
  check(pc.streamCorners > 0, "the engine's indexed stream was read");
  check(pc.streamBad == 0,
        "the engine's indexed stream reads every corner's position, normal and ST as the plain stream");
  check(pc.copyBad == 0, "copyRun of an indexed stream equals the plain stream's (the clip expansion)");
  check(clampedAny, "the clamp poses reach the box limit (the clamped bytes are compared too)");

  std::printf("VU1 arm: %llu packages staged through the VIF decoder and run in both programs, "
              "%d GS words compared, %d differ; controls (a unique moved) caught %llu of %llu\n",
              (unsigned long long)vu.packages, vu.tol.words, vu.tol.wordsDiffering,
              (unsigned long long)vu.controlsCaught, (unsigned long long)vu.controls);
  check(vu.packages > 0 && vu.tol.words > 0, "the VU1 arm compared GS words");
  check(vu.tol.within && vu.tol.wordsDiffering == 0,
        "the indexed program draws exactly what the shipped cull TD draws from the plain stream");
  check(vu.controls > 0 && vu.controlsCaught == vu.controls,
        "the VU1 arm sees a moved unique (every control caught)");

  // Blending in PACKAGE order (the VU0 split's way to skip the gather): the bind data of every
  // package-unique, blended and quantized on its own, is the gathered level-unique result.
  {
    std::mt19937 rngP(0x9ACE5u);
    u64 entries = 0, differ = 0, originDiffer = 0;
    for (auto* group : {&bySize, &sortedBySize})
      for (Case* c : (*group)[48]) {
        const SkelMesh& m = *c->mesh;
        const Ix::Level& L = c->level;
        const size_t E = L.totalUniques();
        std::vector<s16> pq(3 * E);
        std::vector<s8> pn(4 * E);
        std::vector<u8> pj(4 * E), pw(4 * E);
        std::vector<u32> pskin(E);
        for (size_t e = 0; e < E; ++e) {
          const u32 v = c->ub.corner[L.src[e]];
          std::memcpy(&pq[3 * e], &m.pos[3 * (size_t)v], 6);
          std::memcpy(&pn[4 * e], &m.nrm[4 * (size_t)v], 4);
          std::memcpy(&pj[4 * e], &m.joints[4 * (size_t)v], 4);
          std::memcpy(&pw[4 * e], &m.weights[4 * (size_t)v], 4);
          pskin[e] = (u32)e;
        }
        SkelSkin::Unique::UniqueBind pub;
        SkelSkin::Unique::buildUnique(pq.data(), pn.data(), pj.data(), pw.data(), pskin.data(),
                                      (u32)E, bind, pub);
        for (int pose = 0; pose < 10; ++pose) {
          for (u32 j = 0; j < slots; ++j) randomSlot(rngP, &palette[j * 16], 0.3F, 0.1F);
          std::vector<float> a(4 * (size_t)c->ub.count), an(4 * (size_t)c->ub.count);
          std::vector<float> b(4 * E), bn(4 * E);
          SkelSkin::Unique::blend(c->ub, palette.data(), a.data(), an.data());
          SkelSkin::Unique::blend(pub, palette.data(), b.data(), bn.data());
          s32 oa[3], ob[3];
          SkelSkin::Unique::poseOrigin(a.data(), c->ub.count, outBox.step, oa);
          SkelSkin::Unique::poseOrigin(b.data(), (u32)E, outBox.step, ob);
          if (std::memcmp(oa, ob, sizeof oa) != 0) ++originDiffer;
          std::vector<s16> qa(4 * (size_t)c->ub.count), qb(4 * E);
          std::vector<s8> na(4 * (size_t)c->ub.count), nb(4 * E);
          std::vector<u8> ca(c->ub.count), cb(E);
          SkelSkin::Unique::quantize(a.data(), an.data(), c->ub.count, outBox.step, oa, qa.data(),
                                     na.data(), ca.data());
          SkelSkin::Unique::quantize(b.data(), bn.data(), (u32)E, outBox.step, oa, qb.data(),
                                     nb.data(), cb.data());
          for (size_t e = 0; e < E; ++e) {
            ++entries;
            if (std::memcmp(&qb[4 * e], &qa[4 * (size_t)L.src[e]], 8) != 0 ||
                std::memcmp(&nb[4 * e], &na[4 * (size_t)L.src[e]], 4) != 0)
              ++differ;
          }
        }
      }
    std::printf("package-order blend: %llu package-unique entries blended on their own, %llu "
                "differ from the gathered level unique, %llu pose origins differ\n",
                (unsigned long long)entries, (unsigned long long)differ,
                (unsigned long long)originDiffer);
    check(entries > 0 && differ == 0 && originDiffer == 0,
          "a pose blended in package order is bit-equal to the gathered one");
  }

  // The batch kernel arm: its lane model writes the same level-unique arrays.
  {
    const s32 kForced[3] = {0, 0, 1};
    std::vector<float> pal(slots * 16, 0.0F);
    std::mt19937 rngB(0xBA7C4u);
    u64 poses = 0, packagesBad = 0, skipped = 0;
    struct B {
      SkelSkin::Batch::Part part;
    };
    std::vector<B> bp(levels.size());
    u32 qMax = 0;
    for (size_t i = 0; i < levels.size(); ++i) {
      const SkelMesh& m = *levels[i].mesh;
      SkelSkin::Batch::buildPart(m.pos.data(), m.nrm.data(), m.joints.data(), m.weights.data(),
                                 levels[i].src.data(), m.vertexCount, bp[i].part);
      for (u32 v = 0; v < m.vertexCount; ++v)
        for (int a = 0; a < 3; ++a) {
          const u32 q = (u32)std::abs((int)m.pos[3 * v + a]);
          if (q > qMax) qMax = q;
        }
    }
    SkelSkin::Batch::Products prod;
    std::vector<u8> img;
    std::vector<s16> opos;
    std::vector<s8> onrm;
    std::vector<u32> oflags;
    for (int pose = 0; pose < 200; ++pose) {
      for (u32 j = 0; j < slots; ++j) randomSlot(rngB, &pal[j * 16], 0.3F, 0.1F);
      SkelSkin::Fixed::Fold fold;
      SkelSkin::Fixed::fold(pal.data(), slots, bind, outBox, fold, qMax, ~0ULL, kForced);
      if (!(fold.noSat && fold.narrow && fold.narrowN)) {
        ++skipped;
        continue;
      }
      for (size_t i = 0; i < levels.size(); ++i) {
        LevelData& lv = levels[i];
        const SkelSkin::Batch::Part& part = bp[i].part;
        const SkelSkin::Batch::Image* imgp = SkelSkin::Batch::buildImages(fold, part, img);
        if (!SkelSkin::Batch::runPoseFor(fold, part, imgp, prod)) {
          ++skipped;
          continue;
        }
        opos.assign(4 * (size_t)part.unique, 0);
        onrm.assign(4 * (size_t)part.unique, 0);
        oflags.assign(part.unique, 0);
        SkelSkin::Batch::finishPose(fold, part, prod, opos.data(), onrm.data(), oflags.data());
        bool short_ = false;
        for (u32 u = 0; u < part.unique; ++u)
          if (oflags[u] & 2) short_ = true;
        if (short_) {
          ++skipped;
          continue;
        }
        for (Case* c : bySize[48]) {
          if (c->lv != &lv) continue;
          // The scatter's corners: the plain stream's definition of the same uniques.
          std::vector<u8> clamp(part.unique, 0);
          SkelSkin::Unique::UniqueBind& ub = lv.ub;
          check(ub.count == part.unique, "the batch part and the unique bind agree on the uniques");
          std::vector<s16> flatPos((size_t)lv.mesh->vertexCount * 4, 0);
          std::vector<s8> flatNrm((size_t)lv.mesh->vertexCount * 4, 0);
          SkelSkin::Unique::scatterFlat(c->scatter, c->size, ub, opos.data(), onrm.data(),
                                        clamp.data(), lv.mesh->vertexCount, flatPos.data(),
                                        flatNrm.data());
          Ix::writePose(c->level, c->poseLayout, opos.data(), onrm.data(), c->pose.data());
          static std::vector<u32> image(Ix::kHalfQw * 4);
          std::vector<u8> posRun, nrmRun, stRun;
          std::vector<s32> lo, hi, cp;
          const u32 row[4] = {0, 0, 0, 0};
          for (u32 p = 0; p < c->level.packages; ++p) {
            const u32 pcn = c->level.packageCorners(p);
            cp.assign(3 * (size_t)pcn, 0);
            bool ok = decodePackage(*c, p, row, image, posRun, nrmRun, stRun, lo, hi, cp);
            for (u32 k = 0; ok && k < pcn; ++k) {
              const u32 v = p * c->size + k;
              if (std::memcmp(&posRun[(size_t)k * 6], &flatPos[(size_t)v * 4], 6) != 0 ||
                  std::memcmp(&nrmRun[(size_t)k * 4], &flatNrm[(size_t)v * 4], 4) != 0)
                ok = false;
            }
            if (!ok) ++packagesBad;
          }
        }
      }
      ++poses;
    }
    std::printf("batch kernel arm: %llu poses (%llu skipped: the scalar blend takes them), "
                "%llu packages differ\n",
                (unsigned long long)poses, (unsigned long long)skipped,
                (unsigned long long)packagesBad);
    check(poses > 0, "the batch arm ran poses");
    check(packagesBad == 0,
          "the batch kernel's uniques gather and decode to the scatter's corners, byte for byte");
  }

  // (c) the numbers.
  std::printf("\npackaging numbers, soldier model parts (package size = corners a package):\n");
  for (int sorted = 0; sorted < 2; ++sorted) {
    std::printf("%s\n", sorted ? "triangles reordered for locality (orderTriangles):"
                               : "the model's own triangle order:");
    for (u32 size : sizes) {
      std::printf(" package size %u\n", size);
      for (size_t l = 0; l <= 2; ++l) {
        Totals t;
        for (Case* c : (sorted ? sortedBySize : bySize)[size])
          if (c->lv->lod == l) addTotals(t, *c);
        char label[40];
        std::snprintf(label, sizeof label, "fighter LOD%zu (all parts)", l);
        printTotals(label, t);
      }
    }
  }
  std::printf("\nper part level at the shipped package size (48), own order:\n");
  for (Case* c : bySize[48]) {
    Totals t;
    addTotals(t, *c);
    printTotals(c->lv->label, t);
  }

  for (Case* c : cases) delete c;
  std::printf("verify-skel-indexed: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
