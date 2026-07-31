#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace go2_nn_control {

// Constrained NumPy .npy reader for C-order little-endian float32/float64
// arrays with fixed rank and trailing joint dimension.
struct NpyArray {
  std::vector<std::size_t> shape;
  std::vector<double> data;  // always converted to double
};

NpyArray load_npy_f64_matrix(const std::filesystem::path &path,
                             std::size_t expected_cols);

}  // namespace go2_nn_control
