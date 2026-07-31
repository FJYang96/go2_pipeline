#include "go2_nn_control/policy/npy_array.hpp"

#include <zlib.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace go2_nn_control {
namespace {

[[noreturn]] void fail(const std::string &label, const std::string &message) {
  throw std::runtime_error("NPY/NPZ load failed for " + label + ": " + message);
}

template <typename T>
T read_pod_bytes(const char *&cursor, const char *end, const std::string &label) {
  if (static_cast<std::size_t>(end - cursor) < sizeof(T)) {
    fail(label, "unexpected end of buffer");
  }
  T value{};
  std::memcpy(&value, cursor, sizeof(T));
  cursor += sizeof(T);
  return value;
}

NpyArray parse_npy_buffer(const char *begin, const char *end,
                          const std::string &label) {
  const char *cursor = begin;
  const char magic0 = read_pod_bytes<char>(cursor, end, label);
  const char magic1 = read_pod_bytes<char>(cursor, end, label);
  const char magic2 = read_pod_bytes<char>(cursor, end, label);
  const char magic3 = read_pod_bytes<char>(cursor, end, label);
  const char magic4 = read_pod_bytes<char>(cursor, end, label);
  const char magic5 = read_pod_bytes<char>(cursor, end, label);
  if (!(magic0 == '\x93' && magic1 == 'N' && magic2 == 'U' && magic3 == 'M' &&
        magic4 == 'P' && magic5 == 'Y')) {
    fail(label, "invalid magic header");
  }

  const std::uint8_t major = read_pod_bytes<std::uint8_t>(cursor, end, label);
  (void)read_pod_bytes<std::uint8_t>(cursor, end, label);
  std::uint32_t header_len = 0;
  if (major == 1) {
    header_len = read_pod_bytes<std::uint16_t>(cursor, end, label);
  } else if (major == 2 || major == 3) {
    header_len = read_pod_bytes<std::uint32_t>(cursor, end, label);
  } else {
    fail(label, "unsupported npy major version " + std::to_string(major));
  }
  if (static_cast<std::size_t>(end - cursor) < header_len) {
    fail(label, "truncated npy header");
  }
  const std::string header(cursor, cursor + header_len);
  cursor += header_len;

  const std::regex descr_re("'descr'\\s*:\\s*'([^']+)'");
  const std::regex fortran_re("'fortran_order'\\s*:\\s*(True|False)");
  const std::regex shape_re("'shape'\\s*:\\s*\\(([^)]*)\\)");
  std::smatch match;
  if (!std::regex_search(header, match, descr_re)) {
    fail(label, "missing descr field");
  }
  const std::string descr = match[1].str();
  if (!std::regex_search(header, match, fortran_re)) {
    fail(label, "missing fortran_order field");
  }
  if (match[1].str() != "False") {
    fail(label, "only C-order arrays are supported");
  }
  if (!std::regex_search(header, match, shape_re)) {
    fail(label, "missing shape field");
  }

  std::vector<std::size_t> shape;
  {
    std::stringstream shape_stream(match[1].str());
    std::string token;
    while (std::getline(shape_stream, token, ',')) {
      const auto begin_tok = token.find_first_not_of(" \t");
      if (begin_tok == std::string::npos) {
        continue;
      }
      const auto end_tok = token.find_last_not_of(" \t");
      token = token.substr(begin_tok, end_tok - begin_tok + 1);
      if (token.empty()) {
        continue;
      }
      try {
        shape.push_back(static_cast<std::size_t>(std::stoull(token)));
      } catch (const std::exception &) {
        fail(label, "invalid shape entry '" + token + "'");
      }
    }
  }

  const bool is_float32 = (descr == "<f4" || descr == "|f4");
  const bool is_float64 = (descr == "<f8" || descr == "|f8");
  const bool is_int64 = (descr == "<i8" || descr == "|i8");
  if (!is_float32 && !is_float64 && !is_int64) {
    fail(label, "unsupported dtype '" + descr + "'");
  }

  std::size_t count = 1;
  for (const auto dim : shape) {
    count *= dim;
  }

  NpyArray result;
  result.shape = shape;
  if (is_int64) {
    result.dtype = NpyDType::kInt64;
    const std::size_t bytes = count * sizeof(std::int64_t);
    if (static_cast<std::size_t>(end - cursor) < bytes) {
      fail(label, "truncated int64 payload");
    }
    result.int64_data.resize(count);
    std::memcpy(result.int64_data.data(), cursor, bytes);
  } else if (is_float64) {
    result.dtype = NpyDType::kFloat64;
    const std::size_t bytes = count * sizeof(double);
    if (static_cast<std::size_t>(end - cursor) < bytes) {
      fail(label, "truncated float64 payload");
    }
    result.data.resize(count);
    std::memcpy(result.data.data(), cursor, bytes);
  } else {
    result.dtype = NpyDType::kFloat32;
    const std::size_t bytes = count * sizeof(float);
    if (static_cast<std::size_t>(end - cursor) < bytes) {
      fail(label, "truncated float32 payload");
    }
    std::vector<float> buffer(count);
    std::memcpy(buffer.data(), cursor, bytes);
    result.data.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      result.data[i] = static_cast<double>(buffer[i]);
    }
  }
  return result;
}

std::vector<char> inflate_zlib(const char *src, std::size_t src_len,
                               std::size_t expected_size,
                               const std::string &label) {
  std::vector<char> out(expected_size);
  z_stream stream{};
  stream.next_in =
      reinterpret_cast<Bytef *>(const_cast<char *>(src));
  stream.avail_in = static_cast<uInt>(src_len);
  stream.next_out = reinterpret_cast<Bytef *>(out.data());
  stream.avail_out = static_cast<uInt>(out.size());
  // ZIP uses raw deflate (negative window bits).
  if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) {
    fail(label, "inflateInit2 failed");
  }
  const int status = inflate(&stream, Z_FINISH);
  inflateEnd(&stream);
  if (status != Z_STREAM_END) {
    fail(label, "inflate failed");
  }
  if (stream.total_out != expected_size) {
    fail(label, "unexpected inflated size");
  }
  return out;
}

}  // namespace

NpyArray load_npy_from_buffer(const std::vector<char> &buffer,
                              const std::string &label) {
  if (buffer.empty()) {
    fail(label, "empty buffer");
  }
  return parse_npy_buffer(buffer.data(), buffer.data() + buffer.size(), label);
}

NpyArray load_npy_array(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    fail(path.string(), "unable to open file");
  }
  std::vector<char> buffer((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
  return load_npy_from_buffer(buffer, path.string());
}

NpyArray load_npy_f64_matrix(const std::filesystem::path &path,
                             std::size_t expected_cols) {
  auto array = load_npy_array(path);
  if (array.shape.size() != 2) {
    fail(path.string(), "expected rank-2 array");
  }
  if (array.shape[1] != expected_cols) {
    fail(path.string(), "unexpected column count");
  }
  if (array.dtype == NpyDType::kInt64) {
    fail(path.string(), "expected floating-point matrix");
  }
  return array;
}

std::map<std::string, NpyArray> load_npz(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    fail(path.string(), "unable to open npz");
  }
  std::vector<char> file((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  if (file.size() < 30) {
    fail(path.string(), "file too small to be a zip/npz");
  }

  std::map<std::string, NpyArray> arrays;
  std::size_t offset = 0;
  while (offset + 30 <= file.size()) {
    if (std::memcmp(file.data() + offset, "PK\x03\x04", 4) != 0) {
      break;
    }
    const auto compression =
        *reinterpret_cast<const std::uint16_t *>(file.data() + offset + 8);
    const auto compressed_size =
        *reinterpret_cast<const std::uint32_t *>(file.data() + offset + 18);
    const auto uncompressed_size =
        *reinterpret_cast<const std::uint32_t *>(file.data() + offset + 22);
    const auto name_len =
        *reinterpret_cast<const std::uint16_t *>(file.data() + offset + 26);
    const auto extra_len =
        *reinterpret_cast<const std::uint16_t *>(file.data() + offset + 28);
    const std::size_t name_offset = offset + 30;
    if (name_offset + name_len + extra_len > file.size()) {
      fail(path.string(), "truncated zip local header");
    }
    std::string name(file.data() + name_offset, name_len);
    const std::size_t data_offset = name_offset + name_len + extra_len;
    if (data_offset + compressed_size > file.size()) {
      fail(path.string(), "truncated zip payload for " + name);
    }

    std::vector<char> payload;
    if (compression == 0) {
      payload.assign(file.begin() + static_cast<std::ptrdiff_t>(data_offset),
                     file.begin() + static_cast<std::ptrdiff_t>(data_offset +
                                                               compressed_size));
    } else if (compression == 8) {
      payload = inflate_zlib(file.data() + data_offset, compressed_size,
                             uncompressed_size, path.string() + ":" + name);
    } else {
      fail(path.string(), "unsupported zip compression for " + name);
    }

    // Strip directory prefixes and require .npy suffix.
    const auto slash = name.find_last_of('/');
    if (slash != std::string::npos) {
      name = name.substr(slash + 1);
    }
    if (name.size() >= 4 && name.substr(name.size() - 4) == ".npy") {
      name = name.substr(0, name.size() - 4);
    }
    if (!name.empty() && name != ".zgroup" && name.rfind("__", 0) != 0) {
      arrays.emplace(name, load_npy_from_buffer(payload, path.string() + ":" + name));
    }
    offset = data_offset + compressed_size;
  }

  if (arrays.empty()) {
    fail(path.string(), "no arrays found in npz");
  }
  return arrays;
}

}  // namespace go2_nn_control
