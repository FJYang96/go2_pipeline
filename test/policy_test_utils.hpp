#pragma once

#include <openssl/sha.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_policy_test {

inline std::string sha256_file(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("unable to open " + path.string());
  }
  SHA256_CTX context;
  SHA256_Init(&context);
  std::array<char, 1 << 15> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0) {
      SHA256_Update(&context, buffer.data(), static_cast<std::size_t>(count));
    }
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (unsigned char byte : digest) {
    stream << std::setw(2) << static_cast<int>(byte);
  }
  return stream.str();
}

inline void write_manifest(const std::filesystem::path &pkg) {
  const std::vector<std::string> files = {
      "policy.onnx",
      "policy_meta.yaml",
      "reference/metadata.yaml",
      "reference/q_des.npy",
      "reference/qd_des.npy",
      "reference/tau_ff.npy",
  };
  std::ofstream output(pkg / "MANIFEST.sha256");
  for (const auto &rel : files) {
    output << sha256_file(pkg / rel) << "  " << rel << '\n';
  }
}

inline void copy_directory(const std::filesystem::path &from,
                           const std::filesystem::path &to) {
  std::filesystem::remove_all(to);
  std::filesystem::create_directories(to);
  for (const auto &entry :
       std::filesystem::recursive_directory_iterator(from)) {
    const auto relative = std::filesystem::relative(entry.path(), from);
    const auto destination = to / relative;
    if (entry.is_directory()) {
      std::filesystem::create_directories(destination);
    } else if (entry.is_regular_file()) {
      std::filesystem::create_directories(destination.parent_path());
      std::filesystem::copy_file(entry.path(), destination);
    }
  }
}

}  // namespace go2_policy_test
