/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: loader for .tskl skeletal-animation models.
# Binary layout lives in the editor's src/glbparser.cpp (writeTskl) -
# keep both sides in sync. The parse itself is tskl_parse.cpp (host-compilable).
*/

#include "loaders/3d/tskl_loader/tskl_loader.hpp"

#include <stdio.h>
#include <string.h>

#include "debug/debug.hpp"
#include "file/file_utils.hpp"

namespace Tyra {

namespace {

/** Whole file read sequentially into memory (host fs fseek is unreliable). */
bool readWholeFile(const std::string& fullPath, std::vector<u8>& out) {
  FILE* f = fopen(fullPath.c_str(), "rb");
  if (!f) return false;
  out.clear();
  u8 chunk[8192];
  size_t got;
  while ((got = fread(chunk, 1, sizeof(chunk), f)) > 0)
    out.insert(out.end(), chunk, chunk + got);
  fclose(f);
  return !out.empty();
}

/** Adds the .tskl's directory to a texture name that has none. */
void resolveTexture(std::string& name, const std::string& dir) {
  if (!name.empty() && name.find('/') == std::string::npos) name = dir + name;
}

}  // namespace

std::unique_ptr<SkelModel> TsklLoader::load(const std::string& relativePath) {
  std::vector<u8> file;
  if (!readWholeFile(FileUtils::fromCwd(relativePath), file)) {
    TYRA_WARN("TsklLoader: cannot read ", relativePath.c_str());
    return nullptr;
  }
  auto model = std::make_unique<SkelModel>();
  if (!parse(file.data(), file.size(), *model)) {
    TYRA_WARN("TsklLoader: bad or truncated ", relativePath.c_str());
    return nullptr;
  }

  // The directory of the .tskl resolves relative texture names.
  std::string dir = relativePath;
  {
    const size_t slash = dir.find_last_of('/');
    dir = slash == std::string::npos ? "" : dir.substr(0, slash + 1);
  }
  for (SkelPart& part : model->parts) {
    resolveTexture(part.texturePath, dir);
    for (SkelLod& lod : part.lods) resolveTexture(lod.texturePath, dir);
  }
  return model;
}

}  // namespace Tyra
