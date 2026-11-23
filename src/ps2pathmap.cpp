#include "ps2pathmap.hpp"

#include "../vendor/tyra/engine/inc/disc/disc_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace fs = std::filesystem;

namespace ps2pathmap {
namespace {

uint64_t fnv1a(const std::string& s) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (const unsigned char c : s) h = (h ^ c) * 0x100000001b3ull;
    return h;
}

std::string hashName(uint64_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%010llX",
                  (unsigned long long)(v & 0xffffffffffffull));
    return buf;
}

// Opened before the engine loads the map, so these must keep their disc names.
bool staysRaw(const std::string& isoPath) {
    return isoPath == "SYSTEM.CNF" || isoPath == "GYM.RUN" ||
           isoPath == "PS2LINK.RUN" || isoPath == "LAUNCH.SCENE";
}

// FileUtils::fromCwd always appends the ISO9660 version suffix, so the lookup
// key must carry it even though the exporter's own list does not.
std::string cdromOf(const std::string& isoPath) {
    std::string out = "cdrom0:\\";
    for (const char c : isoPath) out += (c == '/') ? '\\' : c;
    if (out.find(';') == std::string::npos) out += ";1";
    return out;
}

void put16(std::vector<uint8_t>& b, size_t& at, uint16_t v) {
    b[at++] = (uint8_t)(v & 0xff);
    b[at++] = (uint8_t)(v >> 8);
}

void put32(std::vector<uint8_t>& b, size_t& at, uint32_t v) {
    b[at++] = (uint8_t)(v & 0xff);
    b[at++] = (uint8_t)((v >> 8) & 0xff);
    b[at++] = (uint8_t)((v >> 16) & 0xff);
    b[at++] = (uint8_t)(v >> 24);
}

struct Mapping {
    std::string logical;
    std::string physical;
    uint32_t logicalOffset = 0;
    uint32_t physicalOffset = 0;
};

}  // namespace

std::string apply(const std::string& projectDir,
                  std::vector<iso9660::FileEntry>& entries, Result* out) {
    if (entries.size() < 2) return "ISO file list has no ELF.";

    std::vector<Mapping> mappings;
    mappings.reserve(entries.size());
    // entries[0] is SYSTEM.CNF; the ELF is entries[1] and keeps its name
    // because SYSTEM.CNF points at it by name.
    for (size_t i = 1; i < entries.size(); ++i) {
        auto& entry = entries[i];
        // SYSTEM.CNF names the ELF, so the ELF must keep its real name too.
        if (i == 1 || staysRaw(entry.isoPath)) continue;

        std::string base = entry.isoPath;
        if (base.size() > 2 && base.compare(base.size() - 2, 2, ";1") == 0)
            base.resize(base.size() - 2);
        const size_t slash = base.find_last_of('/');
        const std::string name =
            slash == std::string::npos ? base : base.substr(slash + 1);
        const size_t dot = name.find_last_of('.');
        const std::string ext = dot == std::string::npos ? "" : name.substr(dot);

        // Hash the WHOLE logical name: two assets of the same kind never share
        // the first 16 identifier characters the console compares. The leading
        // hex digits also bucket them into small directories, each well inside
        // the one sector cdvdman scans.
        const std::string h = hashName(fnv1a(entry.isoPath));
        // No ";1" here: iso9660::write appends the version itself.
        // ONE directory level, not two: with a second bucket level (~170
        // directories) PCSX2's cdvdman could not open a subset of the files -
        // HUD icons, KIT models, combatent textures - even though their
        // directory records and data were valid. Sixteen flat buckets keep
        // every directory far inside one sector and every path openable.
        const std::string physicalRel = "D" + h.substr(0, 1) + "/" + h + ext;
        mappings.push_back({cdromOf(entry.isoPath), cdromOf(physicalRel), 0, 0});
        entry.isoPath = physicalRel;
    }

    if (mappings.size() > Tyra::DiscPath::kMaxEntries)
        return "too many assets for the PS2 path map.";

    // Binary search in the engine compares the logical strings.
    std::sort(mappings.begin(), mappings.end(),
              [](const Mapping& a, const Mapping& b) { return a.logical < b.logical; });

    const size_t entriesOffset = sizeof(Tyra::DiscPath::MapHeader);
    const size_t stringsOffset = entriesOffset + mappings.size() * sizeof(Tyra::DiscPath::MapEntry);
    size_t cursor = stringsOffset;
    for (auto& m : mappings) {
        m.logicalOffset = (uint32_t)cursor;
        cursor += m.logical.size() + 1;
        m.physicalOffset = (uint32_t)cursor;
        cursor += m.physical.size() + 1;
    }
    if (cursor > Tyra::DiscPath::kMaxMapBytes)
        return "PS2 path map exceeds the engine's fixed buffer.";

    std::vector<uint8_t> bytes(cursor, 0);
    size_t at = 0;
    put32(bytes, at, Tyra::DiscPath::kMapMagic);
    put16(bytes, at, Tyra::DiscPath::kMapVersion);
    put16(bytes, at, 0);
    put32(bytes, at, (uint32_t)mappings.size());
    put32(bytes, at, (uint32_t)entriesOffset);
    put32(bytes, at, (uint32_t)stringsOffset);
    for (const auto& m : mappings) {
        put32(bytes, at, m.logicalOffset);
        put32(bytes, at, m.physicalOffset);
    }
    for (const auto& m : mappings) {
        std::memcpy(bytes.data() + m.logicalOffset, m.logical.data(), m.logical.size());
        std::memcpy(bytes.data() + m.physicalOffset, m.physical.data(), m.physical.size());
    }

    const fs::path dir = fs::path(projectDir) / ".res-baked";
    std::error_code ec;
    fs::create_directories(dir, ec);
    const fs::path mapFile = dir / "pathmap.bin";
    std::ofstream outFile(mapFile, std::ios::binary | std::ios::trunc);
    if (!outFile) return "cannot write " + mapFile.string();
    outFile.write((const char*)bytes.data(), (std::streamsize)bytes.size());
    outFile.close();
    if (!outFile) return "cannot write " + mapFile.string();

    entries.insert(entries.begin() + 2, {mapFile, Tyra::DiscPath::kMapIsoName});
    if (out) {
        out->mapFile = mapFile;
        out->mapped = mappings.size();
    }
    return "";
}

}  // namespace ps2pathmap
