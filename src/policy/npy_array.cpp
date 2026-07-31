#include "go2_nn_control/policy/npy_array.hpp"

#include <cstdint>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace go2_nn_control {
namespace {

[[noreturn]] void fail(const std::filesystem::path &path,
                       const std::string &message) {
  throw std::runtime_error("NPY load failed for " + path.string() + ": " +
                           message);
}

template <typename T>
T read_pod(std::ifstream &input, const std::filesystem::path &path) {
  T value{};
  input.read(reinterpret_cast<char *>(&value), sizeof(T));
  if (!input) {
    fail(path, "unexpected end of file while reading header");
  }
  return value;
}

}  // namespace

NpyArray load_npy_f64_matrix(const std::filesystem::path &path,
                             std::size_t expected_cols) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    fail(path, "unable to open file");
  }

  const char magic0 = read_pod<char>(input, path);
  const char magic1 = read_pod<char>(input, path);
  const char magic2 = read_pod<char>(input, path);
  const char magic3 = read_pod<char>(input, path);
  const char magic4 = read_pod<char>(input, path);
  const char magic5 = read_pod<char>(input, path);
  if (!(magic0 == '\x93' && magic1 == 'N' && magic2 == 'U' && magic3 == 'M' &&
        magic4 == 'P' && magic5 == 'Y')) {
    fail(path, "invalid magic header");
  }

  const std::uint8_t major = read_pod<std::uint8_t>(input, path);
  const std::uint8_t minor = read_pod<std::uint8_t>(input, path);
  (void)minor;
  std::uint32_t header_len = 0;
  if (major == 1) {
    header_len = read_pod<std::uint16_t>(input, path);
  } else if (major == 2 || major == 3) {
    header_len = read_pod<std::uint32_t>(input, path);
  } else {
    fail(path, "unsupported npy major version " + std::to_string(major));
  }

  std::string header(header_len, '\0');
  input.read(header.data(), static_cast<std::streamsize>(header_len));
  if (!input) {
    fail(path, "unable to read header dictionary");
  }

  const std::regex descr_re("'descr'\\s*:\\s*'([^']+)'");
  const std::regex fortran_re("'fortran_order'\\s*:\\s*(True|False)");
  const std::regex shape_re("'shape'\\s*:\\s*\\(([^)]*)\\)");
  std::smatch match;

  if (!std::regex_search(header, match, descr_re)) {
    fail(path, "missing descr field");
  }
  const std::string descr = match[1].str();
  if (!std::regex_search(header, match, fortran_re)) {
    fail(path, "missing fortran_order field");
  }
  if (match[1].str() != "False") {
    fail(path, "only C-order arrays are supported");
  }
  if (!std::regex_search(header, match, shape_re)) {
    fail(path, "missing shape field");
  }
  const std::string shape_text = match[1].str();

  std::vector<std::size_t> shape;
  {
    std::stringstream shape_stream(shape_text);
    std::string token;
    while (std::getline(shape_stream, token, ',')) {
      // Trim spaces.
      const auto begin = token.find_first_not_of(" \t");
      if (begin == std::string::npos) {
        continue;
      }
      const auto end = token.find_last_not_of(" \t");
      token = token.substr(begin, end - begin + 1);
      if (token.empty()) {
        continue;
      }
      try {
        shape.push_back(static_cast<std::size_t>(std::stoull(token)));
      } catch (const std::exception &) {
        fail(path, "invalid shape entry '" + token + "'");
      }
    }
  }

  if (shape.size() != 2) {
    fail(path, "expected rank-2 array, got rank " + std::to_string(shape.size()));
  }
  if (shape[1] != expected_cols) {
    fail(path, "expected " + std::to_string(expected_cols) +
                   " columns, got " + std::to_string(shape[1]));
  }

  const bool is_float32 = (descr == "<f4" || descr == "|f4");
  const bool is_float64 = (descr == "<f8" || descr == "|f8");
  if (!is_float32 && !is_float64) {
    fail(path, "unsupported dtype '" + descr + "' (expected <f4 or <f8)");
  }

  const std::size_t rows = shape[0];
  const std::size_t cols = shape[1];
  const std::size_t count = rows * cols;
  NpyArray result;
  result.shape = shape;
  result.data.resize(count);

  if (is_float64) {
    std::vector<double> buffer(count);
    input.read(reinterpret_cast<char *>(buffer.data()),
               static_cast<std::streamsize>(count * sizeof(double)));
    if (!input) {
      fail(path, "unable to read float64 payload");
    }
    result.data = std::move(buffer);
  } else {
    std::vector<float> buffer(count);
    input.read(reinterpret_cast<char *>(buffer.data()),
               static_cast<std::streamsize>(count * sizeof(float)));
    if (!input) {
      fail(path, "unable to read float32 payload");
    }
    for (std::size_t i = 0; i < count; ++i) {
      result.data[i] = static_cast<double>(buffer[i]);
    }
  }

  // Ensure there is no unexpected trailing payload requirement; ignore EOF.
  return result;
}

}  // namespace go2_nn_control
