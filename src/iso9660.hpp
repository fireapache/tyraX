#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

// Minimal ISO9660 image writer for PS2 discs. No Joliet/RockRidge - the PS2
// CD driver (and PCSX2) read plain ISO9660 directory records. The point of
// rolling our own instead of shelling out to mkisofs: file data extents are
// laid out exactly in input order, so the caller controls which assets sit
// next to each other on the disc (adjacent LBAs = no seek between loads).
namespace iso9660 {

struct FileEntry {
    std::filesystem::path source;  // host file to embed
    // Path inside the image, '/'-separated, no leading slash, e.g.
    // "HUD/USE.PNG". Callers pass upper-case names (ISO9660 identifiers);
    // the ";1" file version suffix is appended automatically.
    std::string isoPath;
};

struct PlacedFile {
    std::string isoPath;
    uint32_t lba = 0;   // first sector of the file data
    uint32_t size = 0;  // bytes
};

// Writes the image; files[0] gets the lowest data LBA, files[1] the next,
// and so on. Returns an empty string on success, an error message otherwise.
// outPlacement (optional) receives the final LBA of every file, in layout
// order - useful for reporting the disc layout to the user.
std::string write(const std::filesystem::path& isoFile, const std::string& volumeId,
                  const std::vector<FileEntry>& files,
                  std::vector<PlacedFile>* outPlacement = nullptr);

struct PlannedImage {
    std::vector<PlacedFile> files;  // layout order (same order as the input)
    uint32_t dataStartLba = 0;      // first file-data sector (metadata before it)
    uint32_t totalSectors = 0;      // image size in 2048-byte sectors
};

// Computes the exact same layout write() would produce, without writing
// anything - source files only need to exist to be sized. Backs layout
// previews (the editor's Disc Layout window).
std::string plan(const std::vector<FileEntry>& files, PlannedImage* out);

// The PS2's cdvdman has two hard limits that a plain ISO9660 writer does not
// enforce, and neither produces an obvious error at boot:
//   * it only scans the FIRST 2048-byte sector of a directory extent, so
//     records spilled into later sectors are invisible to fopen();
//   * it matches identifiers by their first 16 characters only, so two names
//     sharing that prefix resolve to whichever record sorts first - a PNG can
//     silently open as a .TSKL and fail in libpng with "Not a PNG file".
// Reports both conditions for the given file set. Returns an error string
// only for a genuinely bad input (same as prepare()); portable constraints
// are returned as human-readable warnings.
std::string validateForPs2(const std::vector<FileEntry>& files,
                           std::vector<std::string>* warnings);

}  // namespace iso9660
