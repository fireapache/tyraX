#include "disc/disc_path.hpp"

#include <cstdio>
#include <cstring>

namespace Tyra::DiscPath {

namespace {

alignas(4) uint8_t buffer[kMaxMapBytes];
const MapEntry* mapEntries = nullptr;
const char* mapStrings = nullptr;
uint32_t mapCount = 0;
bool mapLoaded = false;

}  // namespace

void load() {
  FILE* file = fopen(kMapRawPath, "rb");
  if (!file) return;
  const size_t got = fread(buffer, 1, kMaxMapBytes, file);
  fclose(file);
  // A full buffer means a map too large for the static slot; an old disc has
  // no map at all and simply leaves translation disabled.
  if (got < sizeof(MapHeader) || got >= kMaxMapBytes) return;

  const MapHeader* h = reinterpret_cast<const MapHeader*>(buffer);
  if (h->magic != kMapMagic || h->version != kMapVersion) return;
  if (h->count == 0 || h->count > kMaxEntries) return;
  if (h->entriesOffset < sizeof(MapHeader) ||
      h->entriesOffset + h->count * sizeof(MapEntry) > got)
    return;
  if (h->stringsOffset > got) return;

  mapEntries = reinterpret_cast<const MapEntry*>(buffer + h->entriesOffset);
  // Offsets in the table are absolute file offsets (the writer starts its
  // string cursor at stringsOffset), so the base here is the buffer itself.
  mapStrings = reinterpret_cast<const char*>(buffer);
  mapCount = h->count;
  mapLoaded = true;
}

bool loaded() { return mapLoaded; }

const char* resolve(const char* path) {
  if (!mapLoaded || path == nullptr || strncmp(path, "cdrom", 5) != 0) return path;

  uint32_t lo = 0;
  uint32_t hi = mapCount;
  while (lo < hi) {
    const uint32_t mid = lo + (hi - lo) / 2;
    if (strcmp(mapStrings + mapEntries[mid].logical, path) < 0)
      lo = mid + 1;
    else
      hi = mid;
  }
  if (lo < mapCount && strcmp(mapStrings + mapEntries[lo].logical, path) == 0)
    return mapStrings + mapEntries[lo].physical;
  return path;
}

}  // namespace Tyra::DiscPath
