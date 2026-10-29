// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <openssl/evp.h>
#include <opencv2/core.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace easyfly_orbslam3
{
inline void syncAtlasPath(const std::filesystem::path & path, bool directory = false)
{
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | (directory ? O_DIRECTORY : 0));
  if (fd < 0) {throw std::runtime_error("atlas durability open failed");}
  const bool good = ::fsync(fd) == 0;
  const int closed = ::close(fd);
  if (!good || closed != 0) {throw std::runtime_error("atlas durability sync failed");}
}

// Native ORB removes an existing save target. Keep that operation in the new
// private runtime directory, then exclusively create the user's final file.
inline void publishNewInertialAtlas(
  const std::filesystem::path & native, const std::filesystem::path & target)
{
  if (!std::filesystem::is_regular_file(native) ||
    std::filesystem::file_size(native) > 1024ULL * 1024 * 1024 ||
    !std::filesystem::copy_file(native, target, std::filesystem::copy_options::none))
  {throw std::runtime_error("inertial atlas publication failed");}
  syncAtlasPath(target);
  syncAtlasPath(target.parent_path(), true);
}

inline std::string fileSha256(const std::filesystem::path & path, uintmax_t limit)
{
  if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > limit) {
    throw std::runtime_error("missing or oversized atlas dependency");
  }
  std::ifstream input(path, std::ios::binary);
  std::unique_ptr<EVP_MD_CTX, decltype(& EVP_MD_CTX_free)> digest(EVP_MD_CTX_new(),
    EVP_MD_CTX_free);
  if (!input || !digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize atlas digest");
  }
  std::array<char, 65536> bytes{};
  uintmax_t total = 0;
  while (input) {
    input.read(bytes.data(), bytes.size());const auto size = input.gcount();
    total += size;
    if (total > limit || EVP_DigestUpdate(digest.get(), bytes.data(), size) != 1) {
      throw std::runtime_error("atlas digest read failed");
    }
  }
  if (!input.eof()) {throw std::runtime_error("atlas digest incomplete");}
  std::array<unsigned char, EVP_MAX_MD_SIZE> hash{};unsigned int length = 0;
  if (EVP_DigestFinal_ex(digest.get(), hash.data(), &length) != 1 || length != 32) {
    throw std::runtime_error("atlas digest failed");
  }
  std::ostringstream text;text << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < length; ++i) {
    text << std::setw(2) << unsigned(hash[i]);
  }
  return text.str();
}

inline void verifyInertialAtlas(
  const std::string & stem, const std::string & settings_hash, const std::string & vocabulary_hash)
{
  const auto manifest = stem + ".easyfly.yaml";
  if (!std::filesystem::is_regular_file(manifest) || std::filesystem::file_size(manifest) > 16384) {
    throw std::runtime_error("inertial atlas requires bounded calibration manifest");
  }
  cv::FileStorage file(manifest, cv::FileStorage::READ);
  if (!file.isOpened() || std::string(file["schema"]) != "easyfly-orb-inertial-atlas-1" ||
    std::string(file["sensor_mode"]) != "rgbd_imu" || int(file["workers_joined"]) != 1 ||
    int(file["native_inertial_initialized"]) != 1 || int(file["keyframes"]) < 10 ||
    std::string(file["settings_sha256"]) != settings_hash ||
    std::string(file["vocabulary_sha256"]) != vocabulary_hash ||
    std::string(file["atlas_sha256"]) != fileSha256(stem + ".osa", 1024ULL * 1024 * 1024))
  {throw std::runtime_error("inertial atlas provenance/calibration/readiness mismatch");}
}

inline void saveInertialManifest(
  const std::string & stem, const std::string & settings_hash, const std::string & vocabulary_hash,
  bool initialized, uint64_t keyframes)
{
  cv::FileStorage file("manifest.yaml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
  file << "schema" << "easyfly-orb-inertial-atlas-1" << "sensor_mode" << "rgbd_imu";
  file << "settings_sha256" << settings_hash << "vocabulary_sha256" << vocabulary_hash;
  file << "atlas_sha256" << fileSha256(stem + ".osa", 1024ULL * 1024 * 1024);
  file << "workers_joined" << 1 << "native_inertial_initialized" << int(initialized);
  file << "keyframes" << int(keyframes);
  const auto text = file.releaseAndGetString();
  const auto output = stem + ".easyfly.yaml";
  const int descriptor = ::open(output.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (descriptor < 0) {throw std::runtime_error("atlas manifest cannot overwrite existing output");}
  size_t written = 0;bool good = true;
  while (written < text.size()) {
    const auto count = ::write(descriptor, text.data() + written, text.size() - written);
    if (count <= 0) {good = false;break;} written += count;
  }
  good = good && ::fsync(descriptor) == 0;
  const auto closed = ::close(descriptor);
  if (!good || closed != 0) {throw std::runtime_error("atlas manifest write failed");}
  syncAtlasPath(std::filesystem::path(output).parent_path(), true);
}
}  // namespace easyfly_orbslam3
