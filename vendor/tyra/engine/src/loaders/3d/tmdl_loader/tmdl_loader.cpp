/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: loader for .tmdl binary static models.
# Binary layout lives in the editor's src/tmdl.hpp (written by
# templates::bakeStaticModels) - keep both sides in sync. The parse itself is
# TmdlLoader::parse (tmdl_parse.cpp), which the host verifier builds too.
*/

#include "loaders/3d/tmdl_loader/tmdl_loader.hpp"

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

}  // namespace

std::unique_ptr<LeanObjMesh> TmdlLoader::load(const std::string& relativePath,
                                              bool floats) {
  std::vector<u8> file;
  if (!readWholeFile(FileUtils::fromCwd(relativePath), file)) {
    TYRA_WARN("TmdlLoader: cannot read ", relativePath.c_str());
    return nullptr;
  }
  auto mesh = std::make_unique<LeanObjMesh>();
  if (!parse(file.data(), file.size(), *mesh, floats)) {
    TYRA_WARN("TmdlLoader: bad or truncated model ", relativePath.c_str());
    return nullptr;
  }
  return mesh;
}

}  // namespace Tyra
