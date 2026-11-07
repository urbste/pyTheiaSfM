// theia_convert: convert a binary reconstruction (.recon) to other formats.
//
//   theia_convert --input RECON.recon --output PATH --format FORMAT [flags]
//   theia_convert RECON.recon PATH --format FORMAT [flags]
//
// Formats (same writers as the pyTheia Python API):
//   colmap       PATH is a directory: cameras.txt, images.txt, points3D.txt
//   colmap-bin   PATH is a directory: cameras.bin, images.bin, points3D.bin
//   bundler      PATH is a directory: list.txt and bundle.out
//   nvm          PATH is a .nvm file
//   ply          PATH is a .ply file (points colored by observations)
//   nerfstudio   PATH is a transforms .json; needs --images
//   transforms   PATH is a transforms .json (instant-ngp/gsplat style); needs --images
//   json         PATH is a reconstruction JSON file (WriteReconstructionJson)

#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/io/write_bundler_files.h"
#include "theia/io/write_colmap_files.h"
#include "theia/io/write_nerfstudio.h"
#include "theia/io/write_nvm_file.h"
#include "theia/io/write_ply_file.h"
#include "theia/io/write_transforms_json.h"
#include "theia/sfm/reconstruction.h"

DEFINE_string(input, "", "Input reconstruction (.recon).");
DEFINE_string(output, "", "Output directory or file, depending on --format.");
DEFINE_string(format, "",
              "One of: colmap, colmap-bin, bundler, nvm, ply, nerfstudio, transforms, json.");
DEFINE_string(images, "",
              "Image directory written into nerfstudio/transforms json (required for those).");
DEFINE_int32(aabb_scale, 16, "nerfstudio aabb_scale.");
DEFINE_int32(min_observations, 2, "ply: minimum observations per point.");

namespace {

const char* kFormats[] = {"colmap", "colmap-bin", "bundler", "nvm",
                          "ply",    "nerfstudio",  "transforms", "json"};

bool IsKnownFormat(const std::string& format) {
  for (const char* known : kFormats) {
    if (format == known) {
      return true;
    }
  }
  return false;
}

// Creates the parent directory of a file path, or the directory itself.
bool EnsureDirectory(const std::filesystem::path& directory) {
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  return !ec;
}

bool EnsureParentDirectory(const std::string& file) {
  const std::filesystem::path parent = std::filesystem::absolute(file).parent_path();
  return EnsureDirectory(parent);
}

int Fail(const std::string& message, int code) {
  std::fprintf(stderr, "theia_convert: error: %s\n", message.c_str());
  return code;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::SetUsageMessage(
      std::string("Usage: ") + argv[0] +
      " --input RECON.recon --output PATH --format {colmap,colmap-bin,bundler,nvm,ply,"
      "nerfstudio,transforms,json} [--images DIR] [--aabb_scale N] [--min_observations N]\n"
      "   or: " + argv[0] + " RECON.recon PATH --format FORMAT [flags]\n\n"
      "Converts a binary reconstruction written by WriteReconstruction.");
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  std::string input = FLAGS_input;
  std::string output = FLAGS_output;
  if (argc == 3 && input.empty() && output.empty()) {
    input = argv[1];
    output = argv[2];
  } else if (argc != 1) {
    return Fail("unexpected arguments; use --input and --output (see --help)", 2);
  }

  if (input.empty()) {
    return Fail("--input RECON.recon is required", 2);
  }
  if (output.empty()) {
    return Fail("--output PATH is required", 2);
  }
  if (FLAGS_format.empty()) {
    return Fail("--format is required (see --help)", 2);
  }
  const std::string format = FLAGS_format;
  if (!IsKnownFormat(format)) {
    return Fail("unknown format: " + format, 2);
  }
  if ((format == "nerfstudio" || format == "transforms") && FLAGS_images.empty()) {
    return Fail("--images DIR is required for --format " + format, 2);
  }

  if (!std::filesystem::is_regular_file(input)) {
    return Fail("reconstruction not found: " + input, 1);
  }
  theia::Reconstruction reconstruction;
  bool read_ok = false;
  std::string read_error;
  try {
    read_ok = theia::ReadReconstruction(input, &reconstruction);
  } catch (const std::exception& e) {
    read_error = e.what();
  }
  if (!read_ok) {
    return Fail("could not read reconstruction " + input +
                    " (expected a binary .recon file written by theia::WriteReconstruction)" +
                    (read_error.empty() ? "" : ": " + read_error),
                1);
  }

  bool ok = false;
  if (format == "colmap" || format == "colmap-bin") {
    if (!EnsureDirectory(output)) {
      return Fail("cannot create directory " + output, 1);
    }
    ok = theia::WriteColmapFiles(reconstruction, output, format == "colmap-bin");
  } else if (format == "bundler") {
    if (!EnsureDirectory(output)) {
      return Fail("cannot create directory " + output, 1);
    }
    const std::filesystem::path directory(output);
    ok = theia::WriteBundlerFiles(reconstruction, (directory / "list.txt").string(),
                                  (directory / "bundle.out").string());
  } else if (format == "nvm") {
    if (!EnsureParentDirectory(output)) {
      return Fail("cannot create parent directory of " + output, 1);
    }
    ok = theia::WriteNVMFile(output, reconstruction);
  } else if (format == "ply") {
    if (!EnsureParentDirectory(output)) {
      return Fail("cannot create parent directory of " + output, 1);
    }
    ok = theia::WritePlyFile(output, reconstruction, Eigen::Vector3i(255, 0, 0),
                             FLAGS_min_observations);
  } else if (format == "nerfstudio") {
    if (!EnsureParentDirectory(output)) {
      return Fail("cannot create parent directory of " + output, 1);
    }
    ok = theia::WriteNerfStudio(FLAGS_images, reconstruction, FLAGS_aabb_scale, output);
  } else if (format == "transforms") {
    if (!EnsureParentDirectory(output)) {
      return Fail("cannot create parent directory of " + output, 1);
    }
    ok = theia::WriteTransformsJson(FLAGS_images, reconstruction, output);
  } else {  // json
    if (!EnsureParentDirectory(output)) {
      return Fail("cannot create parent directory of " + output, 1);
    }
    ok = theia::WriteReconstructionJson(reconstruction, output);
  }

  if (!ok) {
    return Fail("writing " + format + " output to " + output + " failed", 1);
  }
  std::printf("wrote %s: %s\n", format.c_str(), output.c_str());
  return 0;
}
