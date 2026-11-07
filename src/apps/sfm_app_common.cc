#include "apps/sfm_app_common.h"

#include <chrono>
#include <cmath>
#include <exception>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <cereal/external/rapidjson/document.h>
#include <cereal/external/rapidjson/prettywriter.h>
#include <cereal/external/rapidjson/stringbuffer.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "apps/options_json.h"
#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/io/write_ply_file.h"
#include "theia/sfm/compute_mean_reprojection_error.h"
#include "theia/sfm/global_reconstruction_estimator.h"
#include "theia/sfm/hybrid_reconstruction_estimator.h"
#include "theia/sfm/incremental_reconstruction_estimator.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/reconstruction_estimator_options.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/random.h"

DEFINE_string(input, "",
              "Directory with reconstruction.recon (views and tracks with features) and "
              "view_graph.bin (ViewGraph::WriteToDisk). Required.");
DEFINE_string(out, "", "Output directory. Required.");
DEFINE_string(view_graph, "",
              "Override the view graph file (default: INPUT/view_graph.bin).");
DEFINE_int64(seed, -1,
             "Fixed RNG seed for the default-constructed random generators. "
             "-1 (default) seeds from the clock.");
DEFINE_string(config, "",
              "JSON object with ReconstructionEstimatorOptions fields. Nested objects are "
              "option groups, e.g. {\"ligt_position_estimator_options\": {\"use_scale_constraint\": false}}.");
DEFINE_bool(list_options, false,
            "List all option names with their default values and exit.");

namespace theia_apps {
namespace {

std::string BaseName(const char* path) {
  return std::filesystem::path(path).filename().string();
}

std::string Usage(const std::string& program, const std::string& method) {
  return "Usage: " + program + " --input DIR --out DIR [--seed N] [--config FILE.json] "
         "[--set key=value ...] [--list_options]\n\n"
         "Runs " + method + " SfM on the tracks and view graph in DIR (the input of "
         "`pytheia sfm`).\n"
         "Writes reconstruction.recon, reconstruction.ply and summary.json into --out.\n"
         "Options are applied in this order: defaults, --config, then each --set.\n"
         "Example: " + program + " --input scene_input --out result --seed 1 "
         "--set max_num_iterations=50\n";
}

std::unique_ptr<theia::ReconstructionEstimator> CreateEstimator(
    theia::ReconstructionEstimatorType method,
    const theia::ReconstructionEstimatorOptions& options) {
  switch (method) {
    case theia::ReconstructionEstimatorType::GLOBAL:
      return std::make_unique<theia::GlobalReconstructionEstimator>(options);
    case theia::ReconstructionEstimatorType::INCREMENTAL:
      return std::make_unique<theia::IncrementalReconstructionEstimator>(options);
    case theia::ReconstructionEstimatorType::HYBRID:
      return std::make_unique<theia::HybridReconstructionEstimator>(options);
  }
  return nullptr;
}

const char* MethodName(theia::ReconstructionEstimatorType method) {
  switch (method) {
    case theia::ReconstructionEstimatorType::GLOBAL:
      return "global";
    case theia::ReconstructionEstimatorType::INCREMENTAL:
      return "incremental";
    case theia::ReconstructionEstimatorType::HYBRID:
      return "hybrid";
  }
  return "unknown";
}

void AddMember(rapidjson::Document* doc, const char* key, rapidjson::Value value) {
  rapidjson::Value name(key, doc->GetAllocator());
  doc->AddMember(name, value, doc->GetAllocator());
}

rapidjson::Value String(const std::string& text, rapidjson::Document* doc) {
  return rapidjson::Value(text.c_str(), static_cast<rapidjson::SizeType>(text.size()),
                          doc->GetAllocator());
}

std::string PrettyJson(const rapidjson::Value& value) {
  rapidjson::StringBuffer buffer;
  // NaN and Infinity are written as NaN / Infinity, as Python's json module does.
  rapidjson::PrettyWriter<rapidjson::StringBuffer, rapidjson::UTF8<>, rapidjson::UTF8<>,
                          rapidjson::CrtAllocator, rapidjson::kWriteNanAndInfFlag>
      writer(buffer);
  writer.SetIndent(' ', 2);
  value.Accept(writer);
  return buffer.GetString();
}

int UsageError(const std::string& program, const std::string& message) {
  std::fprintf(stderr, "%s: error: %s\n", program.c_str(), message.c_str());
  return 2;
}

int RuntimeError(const std::string& program, const std::string& message) {
  std::fprintf(stderr, "%s: error: %s\n", program.c_str(), message.c_str());
  return 1;
}

}  // namespace

int RunSfmApp(int argc, char** argv, theia::ReconstructionEstimatorType method) {
  const std::string program = BaseName(argv[0]);
  const std::string method_name = MethodName(method);

  // --set can be repeated, which gflags does not support; pull it out first.
  std::vector<std::string> set_values;
  std::vector<char*> remaining = ExtractRepeatedFlag("set", argc, argv, &set_values);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  THEIA_GFLAGS_NAMESPACE::SetUsageMessage(Usage(program, method_name));
  int remaining_argc = static_cast<int>(remaining.size());
  char** remaining_argv = remaining.data();
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&remaining_argc, &remaining_argv, true);

  if (remaining_argc > 1) {
    return UsageError(program, std::string("unexpected argument '") + remaining_argv[1] + "'");
  }

  theia::ReconstructionEstimatorOptions options;
  if (FLAGS_list_options) {
    for (const auto& line : ListOptions(options)) {
      std::printf("%s\n", line.c_str());
    }
    return 0;
  }

  if (FLAGS_input.empty()) {
    return UsageError(program, "--input DIR is required (see --help)");
  }
  if (FLAGS_out.empty()) {
    return UsageError(program, "--out DIR is required");
  }
  if (FLAGS_seed < -1) {
    return UsageError(program, "--seed must be a non-negative integer");
  }

  std::string error;
  if (!FLAGS_config.empty() && !ApplyOptionsJsonFile(FLAGS_config, &options, &error)) {
    return UsageError(program, error);
  }
  for (const auto& setting : set_values) {
    if (!ApplyOptionSetting(setting, &options, &error)) {
      return UsageError(program, error);
    }
  }

  const std::filesystem::path input_dir(FLAGS_input);
  const std::string recon_path = (input_dir / "reconstruction.recon").string();
  const std::string graph_path =
      FLAGS_view_graph.empty() ? (input_dir / "view_graph.bin").string() : FLAGS_view_graph;

  if (!std::filesystem::is_regular_file(recon_path)) {
    return RuntimeError(program, "reconstruction not found: " + recon_path);
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
    return RuntimeError(program, "could not read reconstruction " + recon_path +
                                     " (expected a binary .recon file written by "
                                     "theia::WriteReconstruction)" +
                                     (read_error.empty() ? "" : ": " + read_error));
  }

  theia::ViewGraph view_graph;
  bool graph_ok = false;
  try {
    graph_ok = view_graph.ReadFromDisk(graph_path);
  } catch (const std::exception& e) {
    read_error = e.what();
  }
  if (!graph_ok) {
    return RuntimeError(program, "could not read view graph: " + graph_path +
                                     (read_error.empty() ? "" : ": " + read_error));
  }

  // The tracks and the view graph are the input; poses from the file are re-estimated.
  for (const theia::ViewId view_id : reconstruction.ViewIds()) {
    reconstruction.MutableView(view_id)->SetEstimated(false);
  }
  for (const theia::TrackId track_id : reconstruction.TrackIds()) {
    reconstruction.MutableTrack(track_id)->SetEstimated(false);
  }

  if (FLAGS_seed >= 0) {
    theia::SetDefaultRandomSeed(static_cast<unsigned int>(FLAGS_seed));
  }

  std::unique_ptr<theia::ReconstructionEstimator> estimator = CreateEstimator(method, options);
  const auto start = std::chrono::steady_clock::now();
  const theia::ReconstructionEstimatorSummary summary =
      estimator->Estimate(&view_graph, &reconstruction);
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  const std::string out_dir = FLAGS_out;
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    return RuntimeError(program, "cannot create output directory " + out_dir + ": " +
                                     ec.message());
  }

  const bool success = summary.success;
  const double mean_error =
      success ? theia::ComputeMeanReprojectionError(reconstruction) : 0.0;

  rapidjson::Document report;
  report.SetObject();
  AddMember(&report, "method", String(method_name, &report));
  AddMember(&report, "seed",
            FLAGS_seed >= 0 ? rapidjson::Value(static_cast<int64_t>(FLAGS_seed))
                            : rapidjson::Value());
  AddMember(&report, "input",
            String(std::filesystem::absolute(input_dir).lexically_normal().string(), &report));
  AddMember(&report, "success", rapidjson::Value(success));
  AddMember(&report, "message", String(summary.message, &report));
  AddMember(&report, "num_views",
            rapidjson::Value(static_cast<uint64_t>(reconstruction.NumViews())));
  AddMember(&report, "estimated_views",
            rapidjson::Value(static_cast<uint64_t>(summary.estimated_views.size())));
  AddMember(&report, "num_tracks",
            rapidjson::Value(static_cast<uint64_t>(reconstruction.NumTracks())));
  AddMember(&report, "estimated_tracks",
            rapidjson::Value(static_cast<uint64_t>(summary.estimated_tracks.size())));
  AddMember(&report, "mean_reprojection_error_px",
            success ? rapidjson::Value(mean_error) : rapidjson::Value());
  AddMember(&report, "wall_time_s", rapidjson::Value(std::round(elapsed * 1000.0) / 1000.0));

  if (success) {
    const std::string recon_out = (std::filesystem::path(out_dir) / "reconstruction.recon").string();
    if (!theia::WriteReconstruction(reconstruction, recon_out)) {
      return RuntimeError(program, "could not write " + recon_out);
    }
    const std::string ply_out = (std::filesystem::path(out_dir) / "reconstruction.ply").string();
    if (!theia::WritePlyFile(ply_out, reconstruction, Eigen::Vector3i(255, 0, 0), 2)) {
      return RuntimeError(program, "could not write " + ply_out);
    }
  }

  const std::string summary_json = PrettyJson(report);
  {
    const std::string summary_path =
        (std::filesystem::path(out_dir) / "summary.json").string();
    std::ofstream file(summary_path);
    if (!file) {
      return RuntimeError(program, "cannot write " + summary_path);
    }
    file << summary_json << "\n";
  }
  std::printf("%s\n", summary_json.c_str());

  if (!success) {
    std::fprintf(stderr, "%s: estimation failed: %s\n", program.c_str(),
                 summary.message.c_str());
    return 1;
  }
  return 0;
}

}  // namespace theia_apps
