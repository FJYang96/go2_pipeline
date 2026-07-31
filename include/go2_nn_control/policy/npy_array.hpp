#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace go2_nn_control {

enum class NpyDType {
  kFloat32,
  kFloat64,
  kInt64,
};

// Constrained NumPy array container. Floating payloads are converted to double;
// int64 payloads are stored in int64_data.
struct NpyArray {
  std::vector<std::size_t> shape;
  NpyDType dtype = NpyDType::kFloat64;
  std::vector<double> data;
  std::vector<std::int64_t> int64_data;

  std::size_t size() const {
    std::size_t n = 1;
    for (const auto dim : shape) {
      n *= dim;
    }
    return n;
  }
};

NpyArray load_npy_array(const std::filesystem::path &path);
NpyArray load_npy_f64_matrix(const std::filesystem::path &path,
                             std::size_t expected_cols);
NpyArray load_npy_from_buffer(const std::vector<char> &buffer,
                              const std::string &label);

// Minimal .npz reader (ZIP of .npy members). Supports stored and deflate.
std::map<std::string, NpyArray> load_npz(const std::filesystem::path &path);

}  // namespace go2_nn_control
