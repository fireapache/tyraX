#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "iso9660.hpp"

// Rewrites an ISO file list so every asset gets a short, unique, bucketed
// on-disc name that the PS2 cdvdman can actually resolve, and writes the
// logical->physical table the engine loads at boot as PATHMAP.BIN.
namespace ps2pathmap {

struct Result {
    std::filesystem::path mapFile;
    size_t mapped = 0;
};

// Entries[0] is SYSTEM.CNF, entries[1] the ELF; both keep their real names.
// Boot markers opened before the map loads (gym.run, ps2link.run,
// launch.scene) also keep theirs. The generated PATHMAP.BIN is inserted at
// index 2 so it sits near the front of the image.
std::string apply(const std::string& projectDir,
                  std::vector<iso9660::FileEntry>& entries, Result* out);

}  // namespace ps2pathmap
