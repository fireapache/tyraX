#pragma once

#include <cstdint>

// Logical -> physical disc path map, written at the root of an exported ISO as
// PATHMAP.BIN. The PS2 cdvdman matches only the first 16 ISO9660 identifier
// characters and reads only the first sector of a directory extent, so the
// exporter is free to give every asset a short, unique, bucketed on-disc name.
// The engine loads this table once at boot and translates every cdrom0: open
// through it: game code, bakers and content authors keep readable names.
//
// Little-endian, fixed-layout, no heap. Both ends compile THIS header, so the
// binary format cannot drift; kMapVersion rejects a map from another build.
namespace Tyra::DiscPath {

constexpr uint32_t kMapMagic = 0x50414d50;  // "PMAP"
constexpr uint16_t kMapVersion = 1;
constexpr const char* kMapIsoName = "PATHMAP.BIN";  // writer appends ";1"
constexpr const char* kMapRawPath = "cdrom0:\\PATHMAP.BIN;1";
constexpr uint32_t kMaxEntries = 2048;
constexpr uint32_t kMaxMapBytes = 64 * 1024;

struct MapHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t count;
  uint32_t entriesOffset;
  uint32_t stringsOffset;
};

struct MapEntry {
  uint32_t logical;   // offset into the string blob
  uint32_t physical;  // offset into the string blob
};

static_assert(sizeof(MapHeader) == 20, "path map header must be 20 bytes");
static_assert(sizeof(MapEntry) == 8, "path map entry must be 8 bytes");

// Opens the map by a raw root path, so the lookup needs no translation itself.
// A missing or malformed map leaves translation OFF: host / ps2link runs use
// bin/ names unchanged, and an old disc keeps working.
void load();
bool loaded();

// Returns the physical path for a cdrom0: path, or the input when there is no
// map or this path is not in it (boot markers and devkit files pass through).
const char* resolve(const char* cdromPath);

}  // namespace Tyra::DiscPath
