// Copyright (C) 2026, Steffen Urban
// All rights reserved.
//
// See read_colmap_files.h for the license and the conventions.

#include "theia/io/read_colmap_files.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>  // NOLINT
#include <glog/logging.h>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "theia/io/imported_reconstruction.h"
#include "theia/sfm/reconstruction.h"

namespace theia {
namespace {

namespace fs = std::filesystem;
using internal::ImportedIntrinsics;
using internal::ImportedTrack;
using internal::ImportedView;
using internal::ImportError;
using internal::ImportWarning;

// A COLMAP camera model: binary model id and number of parameters
// (colmap/src/colmap/sensor/models.h).
struct ColmapModelInfo {
  std::string name;
  int id;
  int num_params;
};

const std::vector<ColmapModelInfo>& ColmapModels() {
  static const std::vector<ColmapModelInfo> models = {
      {"SIMPLE_PINHOLE", 0, 3},
      {"PINHOLE", 1, 4},
      {"SIMPLE_RADIAL", 2, 4},
      {"RADIAL", 3, 5},
      {"OPENCV", 4, 8},
      {"OPENCV_FISHEYE", 5, 8},
      {"FULL_OPENCV", 6, 12},
      {"FOV", 7, 5},
      {"SIMPLE_RADIAL_FISHEYE", 8, 4},
      {"RADIAL_FISHEYE", 9, 5},
      {"THIN_PRISM_FISHEYE", 10, 12},
      {"RAD_TAN_THIN_PRISM_FISHEYE", 11, 16},
  };
  return models;
}

const ColmapModelInfo* FindModelByName(const std::string& name) {
  for (const ColmapModelInfo& model : ColmapModels()) {
    if (model.name == name) return &model;
  }
  return nullptr;
}

const ColmapModelInfo* FindModelById(const int id) {
  for (const ColmapModelInfo& model : ColmapModels()) {
    if (model.id == id) return &model;
  }
  return nullptr;
}

struct ColmapCamera {
  std::string model;
  int width = 0;
  int height = 0;
  std::vector<double> params;
};

struct ColmapImage {
  uint32_t image_id = 0;
  Eigen::Vector4d qvec;  // w, x, y, z
  Eigen::Vector3d tvec;
  uint32_t camera_id = 0;
  std::string name;
  std::vector<Eigen::Vector2d> xy;
  std::vector<int64_t> point3d_ids;  // -1 when not triangulated
};

struct ColmapPoint {
  Eigen::Vector3d xyz;
  Eigen::Matrix<uint8_t, 3, 1> rgb;
};

struct ColmapModel {
  std::unordered_map<uint32_t, ColmapCamera> cameras;
  std::vector<ColmapImage> images;
  std::map<uint64_t, ColmapPoint> points;  // sorted by point id
};

// ---------------------------------------------------------------------------
// Finding the model folder.
// ---------------------------------------------------------------------------

const char* const kSparseFiles[] = {"cameras", "images", "points3D"};
const int kSparseSearchDepth = 4;

// Returns "bin" or "txt" if directory holds a complete model, else "".
std::string ModelKind(const fs::path& directory) {
  for (const std::string ext : {"bin", "txt"}) {
    bool complete = true;
    for (const char* name : kSparseFiles) {
      std::error_code ec;
      if (!fs::is_regular_file(directory / (std::string(name) + "." + ext),
                               ec)) {
        complete = false;
        break;
      }
    }
    if (complete) return ext;
  }
  return "";
}

// Visits directory and its subfolders depth first in sorted order.
bool SearchModel(const fs::path& directory,
                 const int depth,
                 fs::path* found,
                 std::string* kind) {
  *kind = ModelKind(directory);
  if (!kind->empty()) {
    *found = directory;
    return true;
  }
  if (depth >= kSparseSearchDepth) return false;
  std::vector<fs::path> subdirs;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(directory, ec)) {
    if (entry.is_directory(ec)) subdirs.push_back(entry.path());
  }
  std::sort(subdirs.begin(), subdirs.end());
  for (const fs::path& subdir : subdirs) {
    if (SearchModel(subdir, depth + 1, found, kind)) return true;
  }
  return false;
}

bool FindColmapModel(const std::string& path,
                     fs::path* directory,
                     std::string* kind,
                     ImportMessages* messages) {
  const fs::path root(path);
  std::error_code ec;
  if (!fs::is_directory(root, ec)) {
    return ImportError("COLMAP model directory not found: " + path,
                       messages,
                       /*file_not_found=*/true);
  }
  for (const fs::path& candidate :
       {root, root / "sparse" / "0", root / "sparse", root / "0"}) {
    *kind = ModelKind(candidate);
    if (!kind->empty()) {
      *directory = candidate;
      return true;
    }
  }
  if (SearchModel(root, 0, directory, kind)) return true;
  return ImportError(
      "No COLMAP sparse model (cameras, images, points3D as .txt or .bin) "
      "found under " +
          path,
      messages,
      /*file_not_found=*/true);
}

// ---------------------------------------------------------------------------
// Text format.
// ---------------------------------------------------------------------------

// Reads the lines of a text file without comment lines and line endings.
bool ReadTextLines(const fs::path& file,
                   std::vector<std::string>* lines,
                   ImportMessages* messages) {
  std::ifstream ifs(file);
  if (!ifs.is_open()) {
    return ImportError("Cannot open " + file.string(), messages);
  }
  std::string line;
  while (std::getline(ifs, line)) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
      line.pop_back();
    }
    if (!line.empty() && line[0] == '#') continue;
    lines->push_back(line);
  }
  return true;
}

bool IsBlank(const std::string& line) {
  return std::all_of(line.begin(), line.end(), [](const unsigned char c) {
    return std::isspace(c);
  });
}

std::vector<std::string> SplitWhitespace(const std::string& line) {
  std::istringstream iss(line);
  return std::vector<std::string>(std::istream_iterator<std::string>(iss),
                                  std::istream_iterator<std::string>());
}

// Splits line into max_splits whitespace-separated tokens followed by the
// remainder of the line, like Python's str.split(None, max_splits).
std::vector<std::string> SplitWhitespace(const std::string& line,
                                         const int max_splits) {
  std::vector<std::string> tokens;
  size_t pos = 0;
  auto skip_space = [&]() {
    while (pos < line.size() &&
           std::isspace(static_cast<unsigned char>(line[pos]))) {
      ++pos;
    }
  };
  for (int i = 0; i < max_splits; ++i) {
    skip_space();
    if (pos >= line.size()) return tokens;
    const size_t start = pos;
    while (pos < line.size() &&
           !std::isspace(static_cast<unsigned char>(line[pos]))) {
      ++pos;
    }
    tokens.push_back(line.substr(start, pos - start));
  }
  skip_space();
  if (pos < line.size()) tokens.push_back(line.substr(pos));
  return tokens;
}

bool ParseCamerasText(const fs::path& file,
                      ColmapModel* model,
                      ImportMessages* messages) {
  std::vector<std::string> lines;
  if (!ReadTextLines(file, &lines, messages)) return false;
  for (const std::string& line : lines) {
    if (IsBlank(line)) continue;
    const std::vector<std::string> parts = SplitWhitespace(line);
    if (parts.size() < 4) {
      return ImportError("Malformed line in " + file.string() + ": " + line,
                         messages);
    }
    ColmapCamera camera;
    camera.model = parts[1];
    camera.width = static_cast<int>(std::stod(parts[2]));
    camera.height = static_cast<int>(std::stod(parts[3]));
    for (size_t i = 4; i < parts.size(); ++i) {
      camera.params.push_back(std::stod(parts[i]));
    }
    model->cameras[static_cast<uint32_t>(std::stoul(parts[0]))] = camera;
  }
  return true;
}

bool ParseImagesText(const fs::path& file,
                     ColmapModel* model,
                     ImportMessages* messages) {
  // Each image is two lines: a header and the 2D points. The points line can
  // be empty, so blank lines are kept and consumed in pairs.
  std::vector<std::string> lines;
  if (!ReadTextLines(file, &lines, messages)) return false;
  size_t i = 0;
  while (i < lines.size()) {
    if (IsBlank(lines[i])) {
      ++i;
      continue;
    }
    const std::vector<std::string> header = SplitWhitespace(lines[i], 9);
    const std::string points_line = i + 1 < lines.size() ? lines[i + 1] : "";
    i += 2;
    if (header.size() < 10) {
      return ImportError("Malformed images.txt header: " + lines[i - 2],
                         messages);
    }
    ColmapImage image;
    image.image_id = static_cast<uint32_t>(std::stoul(header[0]));
    for (int k = 0; k < 4; ++k) image.qvec[k] = std::stod(header[1 + k]);
    for (int k = 0; k < 3; ++k) image.tvec[k] = std::stod(header[5 + k]);
    image.camera_id = static_cast<uint32_t>(std::stoul(header[8]));
    image.name = header[9];
    const std::vector<std::string> tokens = SplitWhitespace(points_line);
    if (tokens.size() % 3 != 0) {
      return ImportError("Malformed 2D point list for image " + image.name,
                         messages);
    }
    for (size_t k = 0; k < tokens.size(); k += 3) {
      image.xy.emplace_back(std::stod(tokens[k]), std::stod(tokens[k + 1]));
      image.point3d_ids.push_back(std::stoll(tokens[k + 2]));
    }
    model->images.push_back(std::move(image));
  }
  return true;
}

bool ParsePointsText(const fs::path& file,
                     ColmapModel* model,
                     ImportMessages* messages) {
  std::vector<std::string> lines;
  if (!ReadTextLines(file, &lines, messages)) return false;
  for (const std::string& line : lines) {
    if (IsBlank(line)) continue;
    const std::vector<std::string> parts = SplitWhitespace(line);
    if (parts.size() < 8) {
      return ImportError("Malformed line in " + file.string() + ": " + line,
                         messages);
    }
    ColmapPoint point;
    for (int k = 0; k < 3; ++k) point.xyz[k] = std::stod(parts[1 + k]);
    for (int k = 0; k < 3; ++k) {
      point.rgb[k] = static_cast<uint8_t>(std::stoi(parts[4 + k]));
    }
    model->points[std::stoull(parts[0])] = point;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Binary format (little-endian).
// ---------------------------------------------------------------------------

class BinaryReader {
 public:
  explicit BinaryReader(std::string file) : file_(std::move(file)) {}

  bool Open(ImportMessages* messages) {
    std::ifstream ifs(file_, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
      return ImportError("Cannot open " + file_, messages);
    }
    data_.assign(std::istreambuf_iterator<char>(ifs),
                 std::istreambuf_iterator<char>());
    return true;
  }

  template <typename T>
  bool Read(T* value, ImportMessages* messages) {
    static_assert(std::is_arithmetic<T>::value, "Arithmetic types only.");
    if (offset_ + sizeof(T) > data_.size()) {
      return ImportError("Unexpected end of file in " + file_, messages);
    }
    unsigned char bytes[sizeof(T)];
    std::memcpy(bytes, data_.data() + offset_, sizeof(T));
    if (!IsLittleEndianHost()) std::reverse(bytes, bytes + sizeof(T));
    std::memcpy(value, bytes, sizeof(T));
    offset_ += sizeof(T);
    return true;
  }

  bool ReadNullTerminated(std::string* value, ImportMessages* messages) {
    const auto begin = data_.begin() + offset_;
    const auto end = std::find(begin, data_.end(), '\0');
    if (end == data_.end()) {
      return ImportError("Unterminated image name in " + file_, messages);
    }
    value->assign(begin, end);
    offset_ += value->size() + 1;
    return true;
  }

  bool Skip(const uint64_t num_bytes, ImportMessages* messages) {
    if (offset_ + num_bytes > data_.size()) {
      return ImportError("Unexpected end of file in " + file_, messages);
    }
    offset_ += num_bytes;
    return true;
  }

  bool CheckFullyRead(ImportMessages* messages) const {
    if (offset_ != data_.size()) {
      return ImportError("Unexpected trailing bytes in " + file_, messages);
    }
    return true;
  }

 private:
  static bool IsLittleEndianHost() {
    const uint16_t one = 1;
    unsigned char first_byte;
    std::memcpy(&first_byte, &one, 1);
    return first_byte == 1;
  }

  std::string file_;
  std::vector<char> data_;
  uint64_t offset_ = 0;
};

bool ReadCamerasBinary(const fs::path& file,
                       ColmapModel* model,
                       ImportMessages* messages) {
  BinaryReader reader(file.string());
  if (!reader.Open(messages)) return false;
  uint64_t num_cameras;
  if (!reader.Read(&num_cameras, messages)) return false;
  for (uint64_t i = 0; i < num_cameras; ++i) {
    uint32_t camera_id;
    int32_t model_id;
    uint64_t width, height;
    if (!reader.Read(&camera_id, messages) ||
        !reader.Read(&model_id, messages) || !reader.Read(&width, messages) ||
        !reader.Read(&height, messages)) {
      return false;
    }
    const ColmapModelInfo* info = FindModelById(model_id);
    if (info == nullptr) {
      return ImportError("COLMAP camera model id " + std::to_string(model_id) +
                             " (camera " + std::to_string(camera_id) +
                             ") is not supported.",
                         messages);
    }
    ColmapCamera camera;
    camera.model = info->name;
    camera.width = static_cast<int>(width);
    camera.height = static_cast<int>(height);
    camera.params.resize(info->num_params);
    for (double& param : camera.params) {
      if (!reader.Read(&param, messages)) return false;
    }
    model->cameras[camera_id] = camera;
  }
  return reader.CheckFullyRead(messages);
}

bool ReadImagesBinary(const fs::path& file,
                      ColmapModel* model,
                      ImportMessages* messages) {
  BinaryReader reader(file.string());
  if (!reader.Open(messages)) return false;
  uint64_t num_images;
  if (!reader.Read(&num_images, messages)) return false;
  for (uint64_t i = 0; i < num_images; ++i) {
    ColmapImage image;
    if (!reader.Read(&image.image_id, messages)) return false;
    for (int k = 0; k < 4; ++k) {
      if (!reader.Read(&image.qvec[k], messages)) return false;
    }
    for (int k = 0; k < 3; ++k) {
      if (!reader.Read(&image.tvec[k], messages)) return false;
    }
    uint64_t num_points;
    if (!reader.Read(&image.camera_id, messages) ||
        !reader.ReadNullTerminated(&image.name, messages) ||
        !reader.Read(&num_points, messages)) {
      return false;
    }
    image.xy.resize(num_points);
    image.point3d_ids.resize(num_points);
    for (uint64_t k = 0; k < num_points; ++k) {
      if (!reader.Read(&image.xy[k].x(), messages) ||
          !reader.Read(&image.xy[k].y(), messages) ||
          !reader.Read(&image.point3d_ids[k], messages)) {
        return false;
      }
    }
    model->images.push_back(std::move(image));
  }
  return reader.CheckFullyRead(messages);
}

bool ReadPointsBinary(const fs::path& file,
                      ColmapModel* model,
                      ImportMessages* messages) {
  BinaryReader reader(file.string());
  if (!reader.Open(messages)) return false;
  uint64_t num_points;
  if (!reader.Read(&num_points, messages)) return false;
  for (uint64_t i = 0; i < num_points; ++i) {
    uint64_t point_id, track_length;
    ColmapPoint point;
    double error;
    if (!reader.Read(&point_id, messages)) return false;
    for (int k = 0; k < 3; ++k) {
      if (!reader.Read(&point.xyz[k], messages)) return false;
    }
    for (int k = 0; k < 3; ++k) {
      if (!reader.Read(&point.rgb[k], messages)) return false;
    }
    // The track is read from images.bin; each element is (uint32, uint32).
    if (!reader.Read(&error, messages) ||
        !reader.Read(&track_length, messages) ||
        !reader.Skip(8 * track_length, messages)) {
      return false;
    }
    model->points[point_id] = point;
  }
  return reader.CheckFullyRead(messages);
}

// ---------------------------------------------------------------------------
// Conversion to pyTheia.
// ---------------------------------------------------------------------------

std::string SupportedModelNames() {
  std::string names;
  for (const ColmapModelInfo& model : ColmapModels()) {
    if (!names.empty()) names += ", ";
    names += model.name;
  }
  return names;
}

// Maps a COLMAP camera to pyTheia intrinsics, shifting the principal point by
// pixel_offset.
bool ColmapIntrinsics(const uint32_t camera_id,
                      const ColmapCamera& camera,
                      const double pixel_offset,
                      ImportedIntrinsics* intrinsics,
                      ImportMessages* messages) {
  const ColmapModelInfo* info = FindModelByName(camera.model);
  if (info == nullptr) {
    return ImportError("COLMAP camera model " + camera.model + " (camera " +
                           std::to_string(camera_id) +
                           ") is not supported. Supported: " +
                           SupportedModelNames() + ".",
                       messages);
  }
  if (static_cast<int>(camera.params.size()) != info->num_params) {
    return ImportError("COLMAP camera " + std::to_string(camera_id) + " (" +
                           camera.model + ") has " +
                           std::to_string(camera.params.size()) +
                           " parameters, expected " +
                           std::to_string(info->num_params) + ".",
                       messages);
  }

  const std::vector<double>& p = camera.params;
  const std::string& name = camera.model;
  ImportedIntrinsics& out = *intrinsics;
  out.width = camera.width;
  out.height = camera.height;
  if (name == "SIMPLE_PINHOLE" || name == "SIMPLE_RADIAL" ||
      name == "RADIAL" || name == "SIMPLE_RADIAL_FISHEYE" ||
      name == "RADIAL_FISHEYE") {
    // f, cx, cy followed by the distortion terms.
    out.fx = out.fy = p[0];
    out.cx = p[1];
    out.cy = p[2];
  } else {
    // fx, fy, cx, cy followed by the distortion terms.
    out.fx = p[0];
    out.fy = p[1];
    out.cx = p[2];
    out.cy = p[3];
  }

  if (name == "SIMPLE_PINHOLE" || name == "PINHOLE") {
    out.model_type = CameraIntrinsicsModelType::PINHOLE;
  } else if (name == "SIMPLE_RADIAL") {
    out.model_type = CameraIntrinsicsModelType::PINHOLE;
    out.k1 = p[3];
  } else if (name == "RADIAL") {
    out.model_type = CameraIntrinsicsModelType::PINHOLE;
    out.k1 = p[3];
    out.k2 = p[4];
  } else if (name == "OPENCV") {
    out.model_type = CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL;
    out.k1 = p[4];
    out.k2 = p[5];
    out.p1 = p[6];
    out.p2 = p[7];
  } else if (name == "SIMPLE_RADIAL_FISHEYE") {
    // The same model as RADIAL_FISHEYE with k2 = 0.
    out.model_type = CameraIntrinsicsModelType::RADIAL_FISHEYE;
    out.extra = {p[3], 0.0};
  } else if (name == "RADIAL_FISHEYE") {
    out.model_type = CameraIntrinsicsModelType::RADIAL_FISHEYE;
    out.extra = {p[3], p[4]};
  } else {
    // OPENCV_FISHEYE, FULL_OPENCV, FOV, THIN_PRISM_FISHEYE and
    // RAD_TAN_THIN_PRISM_FISHEYE.
    if (name == "OPENCV_FISHEYE") {
      out.model_type = CameraIntrinsicsModelType::FISHEYE;
    } else if (name == "FULL_OPENCV") {
      out.model_type = CameraIntrinsicsModelType::FULL_OPENCV;
    } else if (name == "FOV") {
      out.model_type = CameraIntrinsicsModelType::FOV;
    } else if (name == "THIN_PRISM_FISHEYE") {
      out.model_type = CameraIntrinsicsModelType::THIN_PRISM_FISHEYE;
    } else {
      out.model_type = CameraIntrinsicsModelType::RAD_TAN_THIN_PRISM_FISHEYE;
    }
    out.extra.assign(p.begin() + 4, p.end());
  }
  out.cx -= pixel_offset;
  out.cy -= pixel_offset;
  return true;
}

bool ReadColmapModel(const fs::path& directory,
                     const std::string& kind,
                     ColmapModel* model,
                     ImportMessages* messages) {
  if (kind == "bin") {
    return ReadCamerasBinary(directory / "cameras.bin", model, messages) &&
           ReadImagesBinary(directory / "images.bin", model, messages) &&
           ReadPointsBinary(directory / "points3D.bin", model, messages);
  }
  try {
    return ParseCamerasText(directory / "cameras.txt", model, messages) &&
           ParseImagesText(directory / "images.txt", model, messages) &&
           ParsePointsText(directory / "points3D.txt", model, messages);
  } catch (const std::exception& e) {
    return ImportError("Malformed COLMAP text model in " + directory.string() +
                           ": " + e.what(),
                       messages);
  }
}

}  // namespace

bool ReadColmapFiles(const std::string& path,
                     Reconstruction* reconstruction,
                     const double pixel_offset,
                     ImportMessages* messages) {
  CHECK_NOTNULL(reconstruction);
  fs::path directory;
  std::string kind;
  if (!FindColmapModel(path, &directory, &kind, messages)) return false;

  ColmapModel model;
  if (!ReadColmapModel(directory, kind, &model, messages)) return false;

  std::sort(model.images.begin(),
            model.images.end(),
            [](const ColmapImage& a, const ColmapImage& b) {
              return a.image_id < b.image_id;
            });

  std::unordered_map<uint32_t, ImportedIntrinsics> intrinsics;
  std::vector<ImportedView> views;
  views.reserve(model.images.size());
  for (const ColmapImage& image : model.images) {
    const auto camera = model.cameras.find(image.camera_id);
    if (camera == model.cameras.end()) {
      return ImportError("Image " + image.name + " refers to unknown camera " +
                             std::to_string(image.camera_id),
                         messages);
    }
    if (intrinsics.count(image.camera_id) == 0 &&
        !ColmapIntrinsics(image.camera_id,
                          camera->second,
                          pixel_offset,
                          &intrinsics[image.camera_id],
                          messages)) {
      return false;
    }
    ImportedView view;
    view.name = image.name;
    view.group_id = image.camera_id;
    view.intrinsics = intrinsics[image.camera_id];
    view.has_pose = true;
    view.rotation = Eigen::Quaterniond(
                        image.qvec[0], image.qvec[1], image.qvec[2],
                        image.qvec[3])
                        .normalized()
                        .toRotationMatrix();
    view.center = -view.rotation.transpose() * image.tvec;
    views.push_back(view);
  }

  std::unordered_map<uint64_t, int> track_index;
  std::vector<ImportedTrack> tracks;
  tracks.reserve(model.points.size());
  for (const auto& [point_id, point] : model.points) {
    track_index[point_id] = static_cast<int>(tracks.size());
    ImportedTrack track;
    track.point = point.xyz;
    track.has_color = true;
    track.color = point.rgb;
    tracks.push_back(track);
  }
  int dropped = 0;
  for (int view_index = 0; view_index < static_cast<int>(model.images.size());
       ++view_index) {
    const ColmapImage& image = model.images[view_index];
    for (size_t k = 0; k < image.xy.size(); ++k) {
      const int64_t point_id = image.point3d_ids[k];
      if (point_id < 0) continue;
      const auto track = track_index.find(static_cast<uint64_t>(point_id));
      if (track == track_index.end()) {
        ++dropped;
        continue;
      }
      tracks[track->second].observations.emplace_back(
          view_index,
          image.xy[k].x() - pixel_offset,
          image.xy[k].y() - pixel_offset);
    }
  }
  if (dropped > 0) {
    ImportWarning(std::to_string(dropped) +
                      " 2D points refer to 3D point ids missing from "
                      "points3D; they were skipped.",
                  messages);
  }

  return internal::BuildImportedReconstruction(
      views, tracks, reconstruction, messages);
}

}  // namespace theia
