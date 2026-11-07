// theia_info: print views, tracks, observations and the mean reprojection error
// of a binary reconstruction (.recon). Output matches `python -m pytheia info`.
//
//   theia_info RECON.recon [--json]

#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "apps/options_json.h"
#include "theia/io/reconstruction_reader.h"
#include "theia/sfm/compute_mean_reprojection_error.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"

DEFINE_bool(json, false, "Print a JSON object instead of text.");

namespace {

std::string EscapeJson(const std::string& text) {
  std::string out;
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::SetUsageMessage(
      std::string("Usage: ") + argv[0] + " RECON.recon [--json]\n\n"
      "Prints the number of views (registered), tracks (estimated), observations\n"
      "and the mean reprojection error of a reconstruction written by WriteReconstruction.");
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 2) {
    std::fprintf(stderr, "theia_info: error: expected exactly one reconstruction file\n");
    std::fprintf(stderr, "%s\n", "Run with --help for usage.");
    return 2;
  }
  const std::string recon_path = argv[1];
  if (!std::filesystem::is_regular_file(recon_path)) {
    std::fprintf(stderr, "theia_info: error: reconstruction not found: %s\n", recon_path.c_str());
    return 1;
  }

  theia::Reconstruction reconstruction;
  bool read_ok = false;
  std::string read_error;
  try {
    read_ok = theia::ReadReconstruction(recon_path, &reconstruction);
  } catch (const std::exception& e) {
    read_error = e.what();
  }
  if (!read_ok) {
    std::fprintf(stderr,
                 "theia_info: error: could not read reconstruction %s (expected a binary "
                 ".recon file written by theia::WriteReconstruction)%s%s\n",
                 recon_path.c_str(), read_error.empty() ? "" : ": ", read_error.c_str());
    return 1;
  }

  const auto view_ids = reconstruction.ViewIds();
  const auto track_ids = reconstruction.TrackIds();
  size_t registered_views = 0;
  size_t observations = 0;
  for (const theia::ViewId view_id : view_ids) {
    const theia::View* view = reconstruction.View(view_id);
    if (view->IsEstimated()) {
      ++registered_views;
    }
    observations += static_cast<size_t>(view->NumFeatures());
  }
  size_t estimated_tracks = 0;
  for (const theia::TrackId track_id : track_ids) {
    if (reconstruction.Track(track_id)->IsEstimated()) {
      ++estimated_tracks;
    }
  }
  const double mean_error = theia::ComputeMeanReprojectionError(reconstruction);
  const std::string absolute_path =
      std::filesystem::absolute(recon_path).lexically_normal().string();

  if (FLAGS_json) {
    std::printf("{\n");
    std::printf("  \"file\": \"%s\",\n", EscapeJson(absolute_path).c_str());
    std::printf("  \"views\": %zu,\n", view_ids.size());
    std::printf("  \"registered_views\": %zu,\n", registered_views);
    std::printf("  \"tracks\": %zu,\n", track_ids.size());
    std::printf("  \"estimated_tracks\": %zu,\n", estimated_tracks);
    std::printf("  \"observations\": %zu,\n", observations);
    // Python's json module spells these NaN and Infinity.
    std::string mean_text = theia_apps::FormatDouble(mean_error);
    if (std::isnan(mean_error)) {
      mean_text = "NaN";
    } else if (std::isinf(mean_error)) {
      mean_text = mean_error < 0 ? "-Infinity" : "Infinity";
    }
    std::printf("  \"mean_reprojection_error_px\": %s\n", mean_text.c_str());
    std::printf("}\n");
    return 0;
  }

  std::printf("%-25s%s\n", "file:", absolute_path.c_str());
  std::printf("%-25s%zu (registered %zu)\n", "views:", view_ids.size(), registered_views);
  std::printf("%-25s%zu (estimated %zu)\n", "tracks:", track_ids.size(), estimated_tracks);
  std::printf("%-25s%zu\n", "observations:", observations);
  std::printf("%-25s%.4f px\n", "mean reprojection error:", mean_error);
  return 0;
}
