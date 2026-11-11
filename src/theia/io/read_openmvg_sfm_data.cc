// Copyright (C) 2026, Steffen Urban
// All rights reserved.
//
// See read_openmvg_sfm_data.h for the license and the conventions.

#include "theia/io/read_openmvg_sfm_data.h"

#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>  // NOLINT
#include <glog/logging.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "theia/io/imported_reconstruction.h"
#include "theia/sfm/reconstruction.h"
#include "theia/util/json.h"

namespace theia {
namespace {

using internal::ImportedIntrinsics;
using internal::ImportedTrack;
using internal::ImportedView;
using internal::ImportError;
using internal::ImportWarning;
using nlohmann::json;

// Distortion keys of the pinhole intrinsics that are read: disto_k1 (one
// radial coefficient), disto_k3 ([k1, k2, k3]) and disto_t2 ([k1, k2, k3, d1,
// d2, t1, t2], Brown).
const char* const kDistortionKeys[] = {"disto_k1", "disto_k3", "disto_t2"};

// A resolved cereal pointer: the polymorphic name and the data.
using Pointee = std::pair<std::string, const json*>;

int64_t ToInt(const json& value) {
  if (value.is_string()) return std::stoll(value.get<std::string>());
  return value.get<int64_t>();
}

int ToIntOrZero(const json& object, const char* key) {
  const auto it = object.find(key);
  if (it == object.end() || it->is_null()) return 0;
  return static_cast<int>(ToInt(*it));
}

// Flattens nested arrays of numbers, like numpy.ravel.
void Flatten(const json& value, std::vector<double>* out) {
  if (value.is_array()) {
    for (const json& element : value) Flatten(element, out);
  } else {
    out->push_back(value.get<double>());
  }
}

std::vector<double> Flatten(const json& value) {
  std::vector<double> out;
  Flatten(value, &out);
  return out;
}

// Records every cereal ptr_wrapper definition (one that has data) by its id.
void CollectPointers(const json& node,
                     std::unordered_map<int64_t, Pointee>* registry) {
  if (node.is_object()) {
    const auto wrapper = node.find("ptr_wrapper");
    if (wrapper != node.end() && wrapper->is_object() &&
        wrapper->contains("data") && wrapper->contains("id")) {
      (*registry)[ToInt((*wrapper)["id"])] =
          Pointee(node.value("polymorphic_name", ""), &(*wrapper)["data"]);
    }
    for (const auto& item : node.items()) CollectPointers(item.value(), registry);
  } else if (node.is_array()) {
    for (const json& element : node) CollectPointers(element, registry);
  }
}

// Resolves an OpenMVG {"polymorphic_id", "ptr_wrapper": {...}} value. A
// ptr_wrapper that holds only an id refers to an earlier definition. Values
// without a ptr_wrapper are returned unchanged.
bool Unwrap(const json& value,
            const std::unordered_map<int64_t, Pointee>& registry,
            Pointee* pointee,
            ImportMessages* messages) {
  if (!value.is_object() || !value.contains("ptr_wrapper")) {
    *pointee = Pointee("", &value);
    return true;
  }
  const json& wrapper = value["ptr_wrapper"];
  if (wrapper.contains("data")) {
    *pointee = Pointee(value.value("polymorphic_name", ""), &wrapper["data"]);
    return true;
  }
  const auto it = wrapper.contains("id") ? registry.find(ToInt(wrapper["id"]))
                                         : registry.end();
  if (it == registry.end()) {
    return ImportError("OpenMVG sfm_data: unresolved pointer id " +
                           wrapper.value("id", json()).dump(),
                       messages);
  }
  *pointee = it->second;
  return true;
}

bool OpenMVGIntrinsics(const std::string& name,
                       const json& data,
                       ImportedIntrinsics* intrinsics,
                       ImportMessages* messages) {
  std::string lower_name = name;
  std::transform(lower_name.begin(),
                 lower_name.end(),
                 lower_name.begin(),
                 [](const unsigned char c) { return std::tolower(c); });
  std::string keys;
  for (const auto& item : data.items()) {
    keys += (keys.empty() ? "" : ", ") + item.key();
  }
  if (lower_name.find("fisheye") != std::string::npos ||
      lower_name.find("spherical") != std::string::npos ||
      data.contains("disto_k4")) {
    return ImportError("OpenMVG intrinsics '" +
                           (name.empty() ? std::string("unnamed") : name) +
                           "' (" + keys + ") are not supported.",
                       messages);
  }
  std::string unknown;
  for (const auto& item : data.items()) {
    const std::string& key = item.key();
    if (key.rfind("disto_", 0) == 0 &&
        std::find(std::begin(kDistortionKeys),
                  std::end(kDistortionKeys),
                  key) == std::end(kDistortionKeys)) {
      unknown += (unknown.empty() ? "" : ", ") + key;
    }
  }
  if (!unknown.empty()) {
    return ImportError(
        "OpenMVG intrinsics use unsupported distortion keys: " + unknown + ".",
        messages);
  }

  ImportedIntrinsics& out = *intrinsics;
  out.fx = out.fy = data.at("focal_length").get<double>();
  const std::vector<double> principal_point = Flatten(data.at("principal_point"));
  if (principal_point.size() != 2) {
    return ImportError("OpenMVG principal_point must hold 2 values.", messages);
  }
  out.cx = principal_point[0];
  out.cy = principal_point[1];
  if (data.contains("disto_k1")) {
    const std::vector<double> coeffs = Flatten(data["disto_k1"]);
    if (coeffs.empty()) {
      return ImportError("OpenMVG disto_k1 is empty.", messages);
    }
    out.k1 = coeffs[0];
  }
  if (data.contains("disto_k3")) {
    const std::vector<double> coeffs = Flatten(data["disto_k3"]);
    if (coeffs.size() != 3) {
      return ImportError("OpenMVG disto_k3 must hold 3 values, got " +
                             std::to_string(coeffs.size()),
                         messages);
    }
    out.k1 = coeffs[0];
    out.k2 = coeffs[1];
    out.k3 = coeffs[2];
  }
  if (data.contains("disto_t2")) {
    const std::vector<double> coeffs = Flatten(data["disto_t2"]);
    if (coeffs.size() != 7) {
      return ImportError("OpenMVG disto_t2 must hold 7 values, got " +
                             std::to_string(coeffs.size()),
                         messages);
    }
    out.k1 = coeffs[0];
    out.k2 = coeffs[1];
    out.k3 = coeffs[2];
    out.p1 = coeffs[5];
    out.p2 = coeffs[6];
    if (coeffs[3] != 0.0 || coeffs[4] != 0.0) {
      ImportWarning(
          "OpenMVG Brown intrinsics: decentering terms d1, d2 were dropped.",
          messages);
    }
  }
  out.model_type = (out.k3 != 0.0 || out.p1 != 0.0 || out.p2 != 0.0)
                       ? CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL
                       : CameraIntrinsicsModelType::PINHOLE;
  out.width = ToIntOrZero(data, "width");
  out.height = ToIntOrZero(data, "height");
  return true;
}

bool ParseSfMData(const json& data,
                  std::vector<ImportedView>* views,
                  std::vector<ImportedTrack>* tracks,
                  ImportMessages* messages) {
  std::unordered_map<int64_t, Pointee> registry;
  CollectPointers(data, &registry);
  const json empty = json::array();
  auto section = [&](const char* key) -> const json& {
    const auto it = data.find(key);
    return it == data.end() || it->is_null() ? empty : *it;
  };

  std::unordered_map<int64_t, ImportedIntrinsics> intrinsics;
  for (const json& entry : section("intrinsics")) {
    Pointee pointee;
    if (!Unwrap(entry.at("value"), registry, &pointee, messages) ||
        !OpenMVGIntrinsics(pointee.first,
                           *pointee.second,
                           &intrinsics[ToInt(entry.at("key"))],
                           messages)) {
      return false;
    }
  }

  std::unordered_map<int64_t, std::pair<Eigen::Matrix3d, Eigen::Vector3d>>
      extrinsics;
  for (const json& entry : section("extrinsics")) {
    const json& value = entry.at("value");
    const std::vector<double> rotation = Flatten(value.at("rotation"));
    const std::vector<double> center = Flatten(value.at("center"));
    if (rotation.size() != 9 || center.size() != 3) {
      return ImportError("OpenMVG extrinsics " + entry.at("key").dump() +
                             " must hold a 3x3 rotation and a 3-vector center.",
                         messages);
    }
    extrinsics[ToInt(entry.at("key"))] = std::make_pair(
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            rotation.data()),
        Eigen::Vector3d(center[0], center[1], center[2]));
  }

  std::vector<std::pair<int64_t, const json*>> raw_views;
  for (const json& entry : section("views")) {
    Pointee pointee;
    if (!Unwrap(entry.at("value"), registry, &pointee, messages)) return false;
    raw_views.emplace_back(ToInt(entry.at("key")), pointee.second);
  }
  std::stable_sort(raw_views.begin(),
                   raw_views.end(),
                   [](const auto& a, const auto& b) { return a.first < b.first; });

  std::unordered_map<int64_t, int> view_index;
  for (const auto& [key, payload_ptr] : raw_views) {
    const json& payload = *payload_ptr;
    const int64_t view_id =
        payload.contains("id_view") ? ToInt(payload["id_view"]) : key;
    const int64_t intrinsic_key = ToInt(payload.at("id_intrinsic"));
    const auto intr = intrinsics.find(intrinsic_key);
    if (intr == intrinsics.end()) {
      return ImportError("OpenMVG view " + payload.value("filename", "") +
                             " refers to missing intrinsics " +
                             std::to_string(intrinsic_key),
                         messages);
    }
    ImportedView view;
    view.group_id = static_cast<CameraIntrinsicsGroupId>(intrinsic_key);
    view.intrinsics = intr->second;
    if (view.intrinsics.width == 0) {
      view.intrinsics.width = ToIntOrZero(payload, "width");
    }
    if (view.intrinsics.height == 0) {
      view.intrinsics.height = ToIntOrZero(payload, "height");
    }
    const auto local_path = payload.find("local_path");
    if (local_path != payload.end() && local_path->is_string() &&
        !local_path->get<std::string>().empty()) {
      view.name = local_path->get<std::string>() + "/";
    }
    view.name += payload.at("filename").get<std::string>();

    const auto pose_key = payload.find("id_pose");
    if (pose_key != payload.end() && !pose_key->is_null()) {
      const auto pose = extrinsics.find(ToInt(*pose_key));
      if (pose != extrinsics.end()) {
        view.has_pose = true;
        view.rotation = pose->second.first;
        view.center = pose->second.second;
      }
    }
    view_index[view_id] = static_cast<int>(views->size());
    views->push_back(view);
  }

  int dropped = 0;
  for (const json& entry : section("structure")) {
    const json& value = entry.at("value");
    ImportedTrack track;
    const std::vector<double> point = Flatten(value.at("X"));
    if (point.size() != 3) {
      return ImportError("OpenMVG structure " + entry.at("key").dump() +
                             " must hold a 3D point.",
                         messages);
    }
    track.point = Eigen::Vector3d(point[0], point[1], point[2]);
    const auto observations = value.find("observations");
    if (observations != value.end()) {
      for (const json& observation : *observations) {
        const json& obs_value = observation.at("value");
        const int64_t obs_view = obs_value.contains("id_view")
                                     ? ToInt(obs_value["id_view"])
                                     : ToInt(observation.at("key"));
        const auto index = view_index.find(obs_view);
        if (index == view_index.end()) {
          ++dropped;
          continue;
        }
        const std::vector<double> xy = Flatten(obs_value.at("x"));
        if (xy.size() != 2) {
          return ImportError("OpenMVG observation must hold 2 coordinates.",
                             messages);
        }
        track.observations.emplace_back(index->second, xy[0], xy[1]);
      }
    }
    const auto color = value.find("color");
    if (color != value.end() && !color->is_null()) {
      const std::vector<double> rgb = Flatten(*color);
      if (rgb.size() != 3) {
        return ImportError("OpenMVG color must hold 3 values.", messages);
      }
      track.has_color = true;
      for (int k = 0; k < 3; ++k) {
        track.color[k] = static_cast<uint8_t>(
            std::clamp(std::round(rgb[k]), 0.0, 255.0));
      }
    }
    tracks->push_back(track);
  }
  if (dropped > 0) {
    ImportWarning(std::to_string(dropped) +
                      " OpenMVG observations refer to unknown views; they "
                      "were skipped.",
                  messages);
  }
  return true;
}

}  // namespace

bool ReadOpenMVGSfMData(const std::string& json_path,
                        Reconstruction* reconstruction,
                        ImportMessages* messages) {
  CHECK_NOTNULL(reconstruction);
  std::error_code ec;
  if (!std::filesystem::is_regular_file(json_path, ec)) {
    return ImportError("OpenMVG sfm_data file not found: " + json_path,
                       messages,
                       /*file_not_found=*/true);
  }
  std::ifstream ifs(json_path);
  if (!ifs.is_open()) {
    return ImportError("Cannot open " + json_path, messages);
  }

  std::vector<ImportedView> views;
  std::vector<ImportedTrack> tracks;
  try {
    const json data = json::parse(ifs);
    if (!ParseSfMData(data, &views, &tracks, messages)) return false;
  } catch (const std::exception& e) {
    return ImportError(
        "Malformed OpenMVG sfm_data " + json_path + ": " + e.what(), messages);
  }
  return internal::BuildImportedReconstruction(
      views, tracks, reconstruction, messages);
}

}  // namespace theia
