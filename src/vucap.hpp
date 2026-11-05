// VU1 packet capture - the host side (docs/devkit.md).
//
// Debugging a VU1 microprogram is blind: you cannot print from VU1 and its
// output goes straight to the GS. Its INPUT, though, is a DMA chain the EE built
// - and the engine's tap can hand one to the editor (bin/vucap.bin). This module
// decodes that chain: the DMA tags, the VIF commands, the UNPACKed data blocks
// (matrices, scales, vertex arrays), and which microprogram the packet started.
// The VIF half is a model of VIF1 itself (decodeVif): what every unpack format
// lands as in VU memory under STROW / STCOL / STMASK / STMOD / STCYCL - the
// compact vertex format's runs included (docs/vu-framework.md, "Compact input").
//
// No GL, no ImGui, no project.hpp - the aobake/livedbg shape.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vucap {

/** One UNPACKed data block: what the EE wrote into VU1 data memory. */
struct Unpack {
    uint32_t vuAddr = 0;      // destination in VU1 data memory (quadwords)
    int count = 0;            // items (VIFcode NUM)
    std::string format;       // "V4_32", "V4_8", ...
    bool useTops = false;     // addressed relative to the double-buffer base
    int program = -1;         // MSCAL entry address this block is run under
    bool masked = false;      // the m bit: each lane per STMASK
    bool usn = false;         // USN: 8/16-bit data zero-extended, else sign
    int mode = 0;             // STMOD in force: 0 normal, 1 offset, 2 difference
    /** False when part of the payload never reached the capture (a REF whose
     * block was not copied): `words` and `floats` are then empty. */
    bool captured = true;
    /** Why this block is NOT counted as a mesh's positions, when it is not.
     * Empty on a position stream. The mesh list is a HEURISTIC over the chain
     * (see load()), and a heuristic that silently drops a stream looks exactly
     * like "the pipeline never sent my model" - so it says why instead. */
    std::string posNote;
    /** What lands in VU memory: four words a quadword written, `count` of
     * them, for EVERY format - 8 and 16-bit lanes sign- or zero-extended (USN),
     * V3-16 packed six bytes an element, STMOD applied (offset: + ROW;
     * difference: + ROW, and ROW takes the sum), each lane of a masked unpack
     * per STMASK (data, ROW, COL of its write cycle, or write-protected - 0
     * here, and flagged in `protect`). A lane the unpack leaves undefined is
     * what PCSX2 writes there: V2 z, w = x, y; V3 w = the next element's x (0
     * past the payload). */
    std::vector<uint32_t> words;
    /** One flag a word of `words`: 1 where STMASK write-protects the lane
     * (mask 3). VU memory keeps what it held there, so whoever writes `words`
     * into a memory image (--vu-replay) must skip it - its 0 is a placeholder,
     * not data. */
    std::vector<uint8_t> protect;
    /** The same lanes as a program reads them: a 32-bit format's word as a
     * float, an 8 / 16-bit DATA lane converted (`itof0` - a compact position's
     * grid index, an ST in 1/4096 units), a ROW / COL lane as the float its
     * bits are (COL's 1.0). */
    std::vector<float> floats;
};

/** VIF1's unpack registers: what STROW, STCOL, STMASK, STMOD and STCYCL left
 * behind. They persist from one VIFcode to the next, across DMA tags. */
struct VifRegs {
    uint32_t row[4] = {0, 0, 0, 0};
    uint32_t col[4] = {0, 0, 0, 0};
    uint32_t mask = 0;
    int mode = 0;        // 0 normal, 1 offset, 2 difference
    int cl = 1, wl = 1;  // STCYCL
};

/** Decodes VIF1 words - VIFcodes and their data, in the order the DMAC hands
 * them to VIF1 - into the unpacks they perform, carrying `regs` across calls.
 * The capture loader runs every chain through this, and
 * src/vucompact_check.hpp holds vugen's compact staging (stageCompact, which
 * restates the format) to what this decodes from the engine's own words, so
 * a capture and the simulator agree on what the VIF writes. `error`, when
 * given, names a VIFcode it cannot size (decoding stops there). */
std::vector<Unpack> decodeVif(const std::vector<uint32_t>& words, VifRegs& regs,
                              std::string* error = nullptr);

/** One mesh of the flush: its position stream, plus what can be said about it
 * without leaving the capture. A flush is a whole bag, so there are as many of
 * these as the bag held - which is why the wireframe needs a selector. */
struct Mesh {
    int unpack = 0;        // index into `unpacks`: the positions
    int verts = 0;
    int tris = 0;
    int program = -1;      // microprogram entry address it runs under
    int degenerate = 0;    // triangles with (near) zero area in model space
    /** Compact positions (docs/compact-vertex-format.md): the bounds and the
     * wireframe are grid indices - the box's step rides in the captured MVP,
     * so the host reference still lands on screen. */
    bool gridUnits = false;
    float lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};  // model-space bounds
    float extent() const;  // longest bbox side
};

/** One decoded line of the chain, for the inspector list. */
struct Step {
    uint32_t offsetQw = 0;
    std::string text;
};

/** One GS vertex the microprogram STAGED for XGKICK: screen space, as the GS
 * will read it (X/Y are 12.4 fixed point, Z is 24-bit). */
struct GsVertex {
    int x = 0, y = 0;      // raw 12.4
    uint32_t z = 0;        // 24-bit
    uint8_t r = 255, g = 255, b = 255, a = 128;
    float s = 0, t = 0, q = 1;
    float px() const { return x / 16.0f; }
    float py() const { return y / 16.0f; }
};

/** A GIF packet found in VU1 memory: the program's OUTPUT. */
struct GifPacket {
    int vuAddr = 0;        // where in VU1 data memory it sits (quadwords)
    int nloop = 0;         // GS vertices
    int nreg = 0;
    bool eop = false;
    bool pre = false;
    uint32_t prim = 0;     // PRIM bits (type, texture, fog, AA, ...)
    std::string regs;      // decoded REGS list, e.g. "ST, RGBAQ, XYZF2"
    bool hasGeometry = false;  // carries XYZF2/XYZ2, i.e. real vertices
    std::vector<GsVertex> verts;
    std::string primName() const;
};

/** What the host computes for the same input, to compare against. */
struct RefVertex {
    float clip[4] = {0, 0, 0, 0};  // after the MVP
    int x = 0, y = 0;              // 12.4, same scaling the program does
    int yFlipped = 0;              // the same Y with the GS axis flipped
    uint32_t z = 0;
    bool behind = false;           // w <= 0: the program's divide blows up here
};

struct Capture {
    bool loaded = false;
    std::string error;
    uint32_t frame = 0;
    int qw = 0;                    // chain length in quadwords
    std::vector<Step> steps;       // decoded DMA tags + VIF commands
    std::vector<Unpack> unpacks;
    std::vector<int> mscal;        // microprogram start addresses seen
    int vertexUnpack = -1;         // index into `unpacks`: the vertex stream
    /** Every mesh of the flush, in send order. A flush carries a whole bag, so
     * a busy frame puts a dozen in here; showing only the biggest one made
     * every capture look identical. */
    std::vector<Mesh> meshes;
    /** Vertices of the biggest position unpack (V4_32 floats, or a compact
     * run's grid indices), as triangles (3 per triangle). */
    const std::vector<float>* vertices() const;
    /** Positions of mesh `i`, same layout. */
    const std::vector<float>* verticesOf(int i) const;
    int triangleCount() const;

    // --- v4: which draw this is, and what the picture measures ---------------
    uint32_t version = 0;
    int flushIndex = -1;   // which bag flush of the frame (-1 = pre-v4 capture)
    int flushCount = 0;    // flushes the last complete frame sent (0 = unknown)
    int renderWidth = 0;   // the game's live render resolution (0 = unknown)
    int renderHeight = 0;

    // --- v3: what VU1 left behind ------------------------------------------
    bool hasVuMem = false;
    std::vector<uint32_t> vuMem;      // 1024 quadwords, as words
    bool hasMvp = false;
    float mvp[16] = {};               // VU1 quadwords 0..3, column per quadword
    float scale[3] = {2048.0f, 2048.0f, 8388607.5f};  // from the packet
    std::vector<GifPacket> gifs;      // the program's staged output
    std::vector<RefVertex> reference;  // host transform of the input vertices
    // Diff of the largest GIF packet against `reference`, in 12.4 units.
    bool yFlipped = false;  // which screen-Y convention matched the hardware
    int diffCompared = 0;
    float diffMaxX = 0, diffMaxY = 0, diffMeanX = 0, diffMeanY = 0;
    int outputVerts() const;
    int inputTris() const;   // triangles the EE sent, over every mesh
    int outputTris() const;  // triangles staged in VU1 memory
    /** Staged triangles minus the LAST mesh's - VU1 memory holds only what the
     * final run(s) left, so this is that mesh's clip fan-out, not the flush's. */
    int clipDelta() const;

    // --- what looks wrong ---------------------------------------------------
    /** The drawing window in GS plane units (the same units GsVertex::px()
     * reports): the engine centres a `renderWidth x renderHeight` window on
     * 2048, so anything outside these bounds is not on screen. Only known when
     * the capture carries the resolution (v4) - and only for the MAIN pass:
     * portal/env/split passes move XYOFFSET while they run. */
    bool hasWindow = false;
    float winX0 = 0, winX1 = 0, winY0 = 0, winY1 = 0;
    int gsVerts = 0;        // vertices of the biggest staged geometry packet
    int gsOffWindow = 0;    // ...of which outside the drawing window
    int gsZeroAlpha = 0;    // ...with alpha 0 in a packet that does not blend
    // That packet's screen bounds, same units as the window. A vertex outside
    // the window is ORDINARY (a triangle crossing the screen edge has two of
    // them, and the GS scissors it) - what is not ordinary is the whole packet
    // missing the window, which is why the finding tests the box, not vertices.
    float gsX0 = 0, gsX1 = 0, gsY0 = 0, gsY1 = 0;
    bool packetOffscreen = false;
    int hugeTris = 0;  // triangles spanning nearly the whole 4096 plane
    int behindVerts = 0;    // input vertices with clip w <= 0 (host reference)
    int degenerateTris = 0; // input triangles with (near) zero area
};

/** Reads and decodes bin/vucap.bin. */
bool load(const std::string& path, Capture& out);
/** The same, for a capture already in memory (the file's bytes). */
bool loadBytes(const std::vector<unsigned char>& bytes, Capture& out);

/** Scans a block of VU1 data memory (1024 quadwords as words) for the GIF
 * packets a microprogram staged, and decodes their vertices.
 *
 * Public because the HOST VU1 SIMULATOR (src/vusim.cpp) ends a run with exactly
 * this memory image, and a simulated run is only comparable to a captured one if
 * both go through the same decoder. Do not reimplement it on the simulator side. */
void scanGifPackets(const std::vector<uint32_t>& vuMem,
                    std::vector<GifPacket>& out);

}  // namespace vucap
