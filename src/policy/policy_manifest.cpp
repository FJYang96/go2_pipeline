#include "go2_nn_control/policy/policy_manifest.hpp"

#include <openssl/sha.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include "go2_nn_control/observation_wrappers.hpp"
#include "yaml-cpp/yaml.h"

namespace go2_nn_control {
namespace {

constexpr double kTimingTolerance = 1.0e-9;
constexpr double kReadyTolerance = 1.0e-9;

const std::vector<std::string> kExpectedJointOrder = {
    "FL_hip_joint",  "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint",  "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint",  "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint",  "RR_thigh_joint", "RR_calf_joint",
};

struct LayoutEntry {
  const char *name;
  int start;
  int dim;
};

constexpr LayoutEntry kExpectedLayout[] = {
    {"rel_quat_wxyz", 0, 4}, {"joint_pos", 4, 12}, {"body_gyro", 16, 3},
    {"joint_vel", 19, 12},   {"phase_cos_sin", 31, 2},
};

const std::vector<std::string> kRequiredManifestAssets = {
    "policy.onnx",
    "policy_meta.yaml",
    "reference/q_des.npy",
    "reference/qd_des.npy",
    "reference/tau_ff.npy",
    "reference/metadata.yaml",
};

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error("PolicyManifest: " + message);
}

std::string require_string(const YAML::Node &node, const std::string &path) {
  if (!node || !node.IsScalar()) {
    fail("missing or invalid string at " + path);
  }
  return node.as<std::string>();
}

bool require_bool(const YAML::Node &node, const std::string &path) {
  if (!node || !node.IsScalar()) {
    fail("missing or invalid bool at " + path);
  }
  return node.as<bool>();
}

double require_double(const YAML::Node &node, const std::string &path) {
  if (!node || !node.IsScalar()) {
    fail("missing or invalid number at " + path);
  }
  const double value = node.as<double>();
  if (!std::isfinite(value)) {
    fail("non-finite number at " + path);
  }
  return value;
}

int require_int(const YAML::Node &node, const std::string &path) {
  if (!node || !node.IsScalar()) {
    fail("missing or invalid integer at " + path);
  }
  return node.as<int>();
}

std::vector<int> require_int_sequence(const YAML::Node &node,
                                      const std::string &path,
                                      std::size_t expected_size) {
  if (!node || !node.IsSequence() ||
      static_cast<std::size_t>(node.size()) != expected_size) {
    fail("expected sequence of length " + std::to_string(expected_size) +
         " at " + path);
  }
  std::vector<int> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < expected_size; ++i) {
    values.push_back(require_int(node[i], path + "[" + std::to_string(i) + "]"));
  }
  return values;
}

std::vector<double> require_double_sequence(const YAML::Node &node,
                                            const std::string &path,
                                            std::size_t expected_size) {
  if (!node || !node.IsSequence() ||
      static_cast<std::size_t>(node.size()) != expected_size) {
    fail("expected sequence of length " + std::to_string(expected_size) +
         " at " + path);
  }
  std::vector<double> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < expected_size; ++i) {
    values.push_back(
        require_double(node[i], path + "[" + std::to_string(i) + "]"));
  }
  return values;
}

std::vector<std::string> require_string_sequence(
    const YAML::Node &node, const std::string &path,
    std::size_t expected_size) {
  if (!node || !node.IsSequence() ||
      static_cast<std::size_t>(node.size()) != expected_size) {
    fail("expected string sequence of length " + std::to_string(expected_size) +
         " at " + path);
  }
  std::vector<std::string> values;
  values.reserve(expected_size);
  for (std::size_t i = 0; i < expected_size; ++i) {
    values.push_back(
        require_string(node[i], path + "[" + std::to_string(i) + "]"));
  }
  return values;
}

JointArray parse_gain(const YAML::Node &node, const std::string &path) {
  JointArray gains{};
  if (!node) {
    fail("missing " + path);
  }
  if (node.IsScalar()) {
    const double value = require_double(node, path);
    if (value < 0.0) {
      fail(path + " must be >= 0");
    }
    gains.fill(value);
    return gains;
  }
  if (node.IsSequence()) {
    const auto values = require_double_sequence(node, path, kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (values[i] < 0.0) {
        fail(path + "[" + std::to_string(i) + "] must be >= 0");
      }
      gains[i] = values[i];
    }
    return gains;
  }
  fail("expected scalar or length-12 sequence at " + path);
}

std::string to_hex(const unsigned char *digest, std::size_t length) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < length; ++i) {
    stream << std::setw(2) << static_cast<int>(digest[i]);
  }
  return stream.str();
}

std::string sha256_file(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    fail("unable to open asset for hashing: " + path.string());
  }
  SHA256_CTX context;
  if (SHA256_Init(&context) != 1) {
    fail("SHA256_Init failed");
  }
  std::array<char, 1 << 16> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0) {
      if (SHA256_Update(&context, buffer.data(),
                        static_cast<std::size_t>(count)) != 1) {
        fail("SHA256_Update failed");
      }
    }
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  if (SHA256_Final(digest, &context) != 1) {
    fail("SHA256_Final failed");
  }
  return to_hex(digest, SHA256_DIGEST_LENGTH);
}

std::filesystem::path resolve_under_policy_dir(
    const std::filesystem::path &policy_dir,
    const std::filesystem::path &relative) {
  if (relative.is_absolute()) {
    fail("manifest path must be package-relative: " + relative.string());
  }
  for (const auto &part : relative) {
    if (part == "..") {
      fail("manifest path must not contain '..': " + relative.string());
    }
  }
  const auto root = std::filesystem::weakly_canonical(policy_dir);
  const auto joined = policy_dir / relative;
  // Prefer weakly_canonical so missing parents still resolve lexically.
  const auto resolved = std::filesystem::weakly_canonical(joined);
  const auto root_string = root.string();
  const auto resolved_string = resolved.string();
  const bool under_root =
      resolved_string == root_string ||
      (resolved_string.size() > root_string.size() &&
       resolved_string.compare(0, root_string.size(), root_string) == 0 &&
       resolved_string[root_string.size()] ==
           std::filesystem::path::preferred_separator);
  if (!under_root) {
    fail("manifest path escapes policy_dir: " + relative.string());
  }
  return resolved;
}

std::map<std::string, std::string> parse_manifest_file(
    const std::filesystem::path &manifest_path) {
  std::ifstream input(manifest_path);
  if (!input) {
    fail("unable to open MANIFEST.sha256");
  }
  std::map<std::string, std::string> entries;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line[0] == '#') {
      continue;
    }
    // sha256sum format: <hex><two spaces or space+*>path
    if (line.size() < 66) {
      fail("malformed MANIFEST.sha256 line " + std::to_string(line_number));
    }
    const std::string digest = line.substr(0, 64);
    if (!std::all_of(digest.begin(), digest.end(), [](char c) {
          return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
                 (c >= 'A' && c <= 'F');
        })) {
      fail("invalid digest on MANIFEST.sha256 line " +
           std::to_string(line_number));
    }
    std::size_t path_start = 64;
    while (path_start < line.size() &&
           (line[path_start] == ' ' || line[path_start] == '\t')) {
      ++path_start;
    }
    if (path_start < line.size() && line[path_start] == '*') {
      ++path_start;
    }
    if (path_start >= line.size()) {
      fail("missing path on MANIFEST.sha256 line " +
           std::to_string(line_number));
    }
    std::string relative = line.substr(path_start);
    // Trim trailing whitespace.
    while (!relative.empty() &&
           (relative.back() == ' ' || relative.back() == '\t' ||
            relative.back() == '\r')) {
      relative.pop_back();
    }
    std::string lower_digest = digest;
    std::transform(lower_digest.begin(), lower_digest.end(),
                   lower_digest.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (!entries.emplace(relative, lower_digest).second) {
      fail("duplicate path in MANIFEST.sha256: " + relative);
    }
  }
  if (entries.empty()) {
    fail("MANIFEST.sha256 contains no entries");
  }
  return entries;
}

void validate_integrity(const std::filesystem::path &policy_dir,
                        const std::map<std::string, std::string> &entries) {
  std::unordered_set<std::string> seen;
  for (const auto &entry : entries) {
    const auto absolute = resolve_under_policy_dir(policy_dir, entry.first);
    if (!std::filesystem::is_regular_file(absolute)) {
      fail("manifest asset missing: " + entry.first);
    }
    const auto actual = sha256_file(absolute);
    if (actual != entry.second) {
      fail("hash mismatch for " + entry.first + " (expected " + entry.second +
           ", got " + actual + ")");
    }
    seen.insert(entry.first);
  }
  for (const auto &required : kRequiredManifestAssets) {
    if (seen.find(required) == seen.end()) {
      fail("MANIFEST.sha256 missing required asset: " + required);
    }
  }
}

void require_equal_string(const std::string &actual, const std::string &expected,
                          const std::string &path) {
  if (actual != expected) {
    fail("expected " + path + " == '" + expected + "', got '" + actual + "'");
  }
}

CompletionBehavior parse_completion_behavior(const std::string &value) {
  if (value == "hold_current") {
    return CompletionBehavior::kHoldCurrent;
  }
  if (value == "move_to_neutral") {
    return CompletionBehavior::kMoveToNeutral;
  }
  fail("unsupported execution.completion_behavior '" + value + "'");
}

}  // namespace

std::filesystem::path PolicyManifest::onnx_path() const {
  return policy_dir_ / "policy.onnx";
}

std::filesystem::path PolicyManifest::reference_q_path() const {
  return policy_dir_ / "reference" / "q_des.npy";
}

std::filesystem::path PolicyManifest::reference_qd_path() const {
  return policy_dir_ / "reference" / "qd_des.npy";
}

std::filesystem::path PolicyManifest::reference_tau_path() const {
  return policy_dir_ / "reference" / "tau_ff.npy";
}

std::filesystem::path PolicyManifest::reference_metadata_path() const {
  return policy_dir_ / "reference" / "metadata.yaml";
}

std::filesystem::path PolicyManifest::policy_meta_path() const {
  return policy_dir_ / "policy_meta.yaml";
}

const char *PolicyManifest::completion_behavior_string() const {
  switch (completion_behavior_) {
    case CompletionBehavior::kHoldCurrent:
      return "hold_current";
    case CompletionBehavior::kMoveToNeutral:
      return "move_to_neutral";
  }
  return "unknown";
}

PolicyManifest PolicyManifest::Load(const std::filesystem::path &policy_dir) {
  if (!std::filesystem::is_directory(policy_dir)) {
    fail("policy_dir is not a directory: " + policy_dir.string());
  }

  PolicyManifest manifest;
  manifest.policy_dir_ = std::filesystem::weakly_canonical(policy_dir);

  const auto manifest_path = manifest.policy_dir_ / "MANIFEST.sha256";
  if (!std::filesystem::is_regular_file(manifest_path)) {
    fail("missing MANIFEST.sha256");
  }
  if (!std::filesystem::is_regular_file(manifest.policy_meta_path())) {
    fail("missing policy_meta.yaml");
  }

  const auto entries = parse_manifest_file(manifest_path);
  validate_integrity(manifest.policy_dir_, entries);

  YAML::Node root;
  try {
    root = YAML::LoadFile(manifest.policy_meta_path().string());
  } catch (const std::exception &error) {
    fail(std::string("failed to parse policy_meta.yaml: ") + error.what());
  }
  if (!root || !root.IsMap()) {
    fail("policy_meta.yaml root must be a mapping");
  }

  if (require_int(root["schema_version"], "schema_version") != 1) {
    fail("unsupported schema_version");
  }

  const auto model = root["model"];
  if (!model || !model.IsMap()) {
    fail("missing model block");
  }
  manifest.model_input_name_ =
      require_string(model["input_name"], "model.input_name");
  manifest.model_output_name_ =
      require_string(model["output_name"], "model.output_name");
  require_equal_string(manifest.model_input_name_, "obs", "model.input_name");
  require_equal_string(manifest.model_output_name_, "actions",
                       "model.output_name");
  const auto input_shape =
      require_int_sequence(model["input_shape"], "model.input_shape", 2);
  const auto output_shape =
      require_int_sequence(model["output_shape"], "model.output_shape", 2);
  if (input_shape[0] != 1 || input_shape[1] != 33) {
    fail("model.input_shape must be [1, 33]");
  }
  if (output_shape[0] != 1 || output_shape[1] != 12) {
    fail("model.output_shape must be [1, 12]");
  }
  require_equal_string(require_string(model["input_dtype"], "model.input_dtype"),
                       "float32", "model.input_dtype");
  require_equal_string(
      require_string(model["output_dtype"], "model.output_dtype"), "float32",
      "model.output_dtype");

  const auto observation = root["observation"];
  if (!observation || !observation.IsMap()) {
    fail("missing observation block");
  }
  if (require_int(observation["num_obs"], "observation.num_obs") != 33) {
    fail("observation.num_obs must be 33");
  }
  require_equal_string(
      require_string(observation["quaternion_format"],
                     "observation.quaternion_format"),
      "wxyz", "observation.quaternion_format");
  require_equal_string(
      require_string(observation["quaternion_frame"],
                     "observation.quaternion_frame"),
      "episode_relative_imu", "observation.quaternion_frame");
  require_equal_string(
      require_string(observation["quaternion_sign_convention"],
                     "observation.quaternion_sign_convention"),
      "nonnegative_w", "observation.quaternion_sign_convention");
  require_equal_string(
      require_string(observation["gyro_frame"], "observation.gyro_frame"),
      "body", "observation.gyro_frame");
  require_equal_string(
      require_string(observation["phase_encoding"],
                     "observation.phase_encoding"),
      "cos_sin_normalized_horizon", "observation.phase_encoding");

  const auto layout = observation["layout"];
  constexpr std::size_t expected_layout_size =
      sizeof(kExpectedLayout) / sizeof(kExpectedLayout[0]);
  if (!layout || !layout.IsSequence() ||
      static_cast<std::size_t>(layout.size()) != expected_layout_size) {
    fail("observation.layout must contain the five fixed blocks");
  }
  for (std::size_t i = 0; i < expected_layout_size; ++i) {
    const auto entry = layout[static_cast<int>(i)];
    if (!entry || !entry.IsMap()) {
      fail("invalid observation.layout entry");
    }
    require_equal_string(require_string(entry["name"], "observation.layout.name"),
                         kExpectedLayout[i].name, "observation.layout.name");
    if (require_int(entry["start"], "observation.layout.start") !=
        kExpectedLayout[i].start) {
      fail("unexpected observation.layout start for " +
           std::string(kExpectedLayout[i].name));
    }
    if (require_int(entry["dim"], "observation.layout.dim") !=
        kExpectedLayout[i].dim) {
      fail("unexpected observation.layout dim for " +
           std::string(kExpectedLayout[i].name));
    }
  }

  const auto action = root["action"];
  if (!action || !action.IsMap()) {
    fail("missing action block");
  }
  if (require_int(action["num_actions"], "action.num_actions") != 12) {
    fail("action.num_actions must be 12");
  }
  require_equal_string(require_string(action["space"], "action.space"),
                       "tanh_normalized", "action.space");
  if (require_bool(action["scale_baked_into_onnx"],
                   "action.scale_baked_into_onnx")) {
    fail("action.scale_baked_into_onnx must be false");
  }
  manifest.position_correction_limit_rad_ = require_double(
      action["position_correction_limit_rad"],
      "action.position_correction_limit_rad");
  if (!(manifest.position_correction_limit_rad_ > 0.0)) {
    fail("action.position_correction_limit_rad must be > 0");
  }

  const auto execution = root["execution"];
  if (!execution || !execution.IsMap()) {
    fail("missing execution block");
  }
  require_equal_string(require_string(execution["mode"], "execution.mode"),
                       "one_shot", "execution.mode");
  require_equal_string(
      require_string(execution["reference_sampling"],
                     "execution.reference_sampling"),
      "zero_order_hold", "execution.reference_sampling");
  manifest.control_dt_ =
      require_double(execution["control_dt"], "execution.control_dt");
  if (!(manifest.control_dt_ > 0.0)) {
    fail("execution.control_dt must be > 0");
  }
  const int horizon = require_int(execution["horizon_N"], "execution.horizon_N");
  if (horizon <= 0) {
    fail("execution.horizon_N must be > 0");
  }
  manifest.horizon_ = static_cast<std::size_t>(horizon);
  manifest.duration_seconds_ =
      require_double(execution["duration_s"], "execution.duration_s");
  if (!(manifest.duration_seconds_ > 0.0)) {
    fail("execution.duration_s must be > 0");
  }
  const double expected_duration =
      manifest.control_dt_ * static_cast<double>(manifest.horizon_);
  if (std::abs(manifest.duration_seconds_ - expected_duration) >
      kTimingTolerance) {
    fail("execution.duration_s must equal control_dt * horizon_N");
  }
  manifest.completion_behavior_ = parse_completion_behavior(require_string(
      execution["completion_behavior"], "execution.completion_behavior"));

  const auto pd_gains = root["pd_gains"];
  if (!pd_gains || !pd_gains.IsMap()) {
    fail("missing pd_gains block");
  }
  manifest.policy_kp_ = parse_gain(pd_gains["kp"], "pd_gains.kp");
  manifest.policy_kd_ = parse_gain(pd_gains["kd"], "pd_gains.kd");

  manifest.joint_order_ =
      require_string_sequence(root["joint_order"], "joint_order", kJointCount);
  if (manifest.joint_order_ != kExpectedJointOrder) {
    fail("unsupported joint_order");
  }

  const auto normalization = root["normalization"];
  if (!normalization || !normalization.IsMap()) {
    fail("missing normalization block");
  }
  if (!require_bool(normalization["baked_into_onnx"],
                    "normalization.baked_into_onnx")) {
    fail("normalization.baked_into_onnx must be true");
  }
  manifest.normalizer_eps_ =
      require_double(normalization["eps"], "normalization.eps");
  if (!(manifest.normalizer_eps_ >= 0.0)) {
    fail("normalization.eps must be >= 0");
  }
  manifest.normalizer_mean_ = require_double_sequence(
      normalization["mean"], "normalization.mean", kObservationSize);
  manifest.normalizer_std_ = require_double_sequence(
      normalization["std"], "normalization.std", kObservationSize);

  const auto ready = root["ready_position"];
  if (!ready || !ready.IsMap()) {
    fail("missing ready_position block");
  }
  require_equal_string(
      require_string(ready["joint_order"], "ready_position.joint_order"),
      "policy", "ready_position.joint_order");
  const auto ready_values = require_double_sequence(
      ready["values"], "ready_position.values", kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    manifest.ready_position_policy_[i] = ready_values[i];
  }
  if (ready["unitree_order_values"]) {
    const auto unitree_values = require_double_sequence(
        ready["unitree_order_values"], "ready_position.unitree_order_values",
        kJointCount);
    JointArray unitree{};
    for (std::size_t i = 0; i < kJointCount; ++i) {
      unitree[i] = unitree_values[i];
    }
    const auto expected_unitree =
        reorder_policy_to_unitree(manifest.ready_position_policy_);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (std::abs(unitree[i] - expected_unitree[i]) > kReadyTolerance) {
        fail("ready_position.unitree_order_values does not match policy order "
             "conversion");
      }
    }
  }

  // Provenance is opaque; capture stringifiable scalars for inspection only.
  if (root["provenance"] && root["provenance"].IsMap()) {
    for (const auto &item : root["provenance"]) {
      const auto key = item.first.as<std::string>();
      if (item.second && item.second.IsScalar()) {
        manifest.provenance_[key] = item.second.as<std::string>();
      }
    }
  }

  // Cross-check reference metadata using canonical package-relative path only.
  YAML::Node reference_meta;
  try {
    reference_meta =
        YAML::LoadFile(manifest.reference_metadata_path().string());
  } catch (const std::exception &error) {
    fail(std::string("failed to parse reference/metadata.yaml: ") +
         error.what());
  }
  if (!reference_meta || !reference_meta.IsMap()) {
    fail("reference/metadata.yaml root must be a mapping");
  }
  if (require_int(reference_meta["schema_version"],
                  "reference.schema_version") != 1) {
    fail("unsupported reference/metadata.yaml schema_version");
  }
  const double ref_dt =
      require_double(reference_meta["control_dt"], "reference.control_dt");
  if (std::abs(ref_dt - manifest.control_dt_) > kTimingTolerance) {
    fail("reference/metadata.yaml control_dt does not match policy_meta");
  }
  const int ref_horizon =
      require_int(reference_meta["horizon_N"], "reference.horizon_N");
  if (static_cast<std::size_t>(ref_horizon) != manifest.horizon_) {
    fail("reference/metadata.yaml horizon_N does not match policy_meta");
  }
  const auto ref_joint_order = require_string_sequence(
      reference_meta["joint_order"], "reference.joint_order", kJointCount);
  if (ref_joint_order != manifest.joint_order_) {
    fail("reference/metadata.yaml joint_order does not match policy_meta");
  }

  return manifest;
}

}  // namespace go2_nn_control
