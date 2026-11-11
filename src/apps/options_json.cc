#include "apps/options_json.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <sstream>
#include <utility>

#include <cereal/external/rapidjson/document.h>
#include <cereal/external/rapidjson/error/en.h>

namespace theia_apps {
namespace {

using theia::ReconstructionEstimatorOptions;

// ----------------------------------------------------------------------------
// Text helpers
// ----------------------------------------------------------------------------

std::string Trim(const std::string& text) {
  size_t begin = 0;
  size_t end = text.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(text[begin]))) {
    ++begin;
  }
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
    --end;
  }
  return text.substr(begin, end - begin);
}

std::string Upper(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return text;
}

std::string Lower(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

// Parses a finite or non-finite double; the whole string must be consumed.
bool ParseDouble(const std::string& text, double* out) {
  const std::string t = Trim(text);
  if (t.empty()) {
    return false;
  }
  char* end = nullptr;
  const double value = std::strtod(t.c_str(), &end);
  if (end != t.c_str() + t.size()) {
    return false;
  }
  *out = value;
  return true;
}

// Integers are accepted with or without a fractional zero ("50" or "50.0"),
// matching the Python CLI.
bool ParseInt(const std::string& text, int* out) {
  double value = 0.0;
  if (!ParseDouble(text, &value)) {
    return false;
  }
  if (!std::isfinite(value) || value != std::floor(value) ||
      value < static_cast<double>(INT_MIN) || value > static_cast<double>(INT_MAX)) {
    return false;
  }
  *out = static_cast<int>(value);
  return true;
}

bool ParseBool(const std::string& text, bool* out) {
  const std::string t = Lower(Trim(text));
  if (t == "1" || t == "true" || t == "yes" || t == "on") {
    *out = true;
    return true;
  }
  if (t == "0" || t == "false" || t == "no" || t == "off") {
    *out = false;
    return true;
  }
  return false;
}

}  // namespace

std::string FormatDouble(double value) {
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return value < 0.0 ? "-inf" : "inf";
  }
  // Shortest scientific form that round-trips, e.g. "1.5e+02".
  char buffer[64];
  for (int precision = 0; precision <= 16; ++precision) {
    std::snprintf(buffer, sizeof(buffer), "%.*e", precision, value);
    if (std::strtod(buffer, nullptr) == value) {
      break;
    }
  }
  std::string sci = buffer;
  const bool negative = !sci.empty() && sci[0] == '-';
  if (negative) {
    sci.erase(0, 1);
  }
  const size_t e_pos = sci.find('e');
  std::string digits;
  for (size_t i = 0; i < e_pos; ++i) {
    if (std::isdigit(static_cast<unsigned char>(sci[i]))) {
      digits += sci[i];
    }
  }
  const int exponent = std::atoi(sci.c_str() + e_pos + 1);

  // Python repr layout: fixed notation for 1e-4 <= |x| < 1e16, otherwise scientific.
  std::string text;
  if (exponent >= -4 && exponent < 16) {
    if (exponent >= 0) {
      const size_t int_len = static_cast<size_t>(exponent) + 1;
      std::string int_part = digits.substr(0, std::min(digits.size(), int_len));
      while (int_part.size() < int_len) {
        int_part += '0';
      }
      const std::string frac = digits.size() > int_len ? digits.substr(int_len) : "0";
      text = int_part + "." + frac;
    } else {
      text = "0." + std::string(static_cast<size_t>(-exponent - 1), '0') + digits;
    }
  } else {
    text = digits.substr(0, 1);
    if (digits.size() > 1) {
      text += "." + digits.substr(1);
    }
    std::snprintf(buffer, sizeof(buffer), "e%c%02d", exponent < 0 ? '-' : '+',
                  std::abs(exponent));
    text += buffer;
  }
  return negative ? "-" + text : text;
}

namespace {

std::string Join(const std::vector<std::string>& parts, const std::string& sep) {
  std::string result;
  for (size_t i = 0; i < parts.size(); ++i) {
    if (i > 0) {
      result += sep;
    }
    result += parts[i];
  }
  return result;
}

// ----------------------------------------------------------------------------
// Option fields: one entry per settable leaf, keyed by its dotted name.
// ----------------------------------------------------------------------------

struct Field {
  std::string name;
  // Parses user text (from JSON or --set) into the option. Sets *error on failure.
  std::function<bool(const std::string& text, std::string* error)> parse;
  // Current value for listings.
  std::function<std::string()> format;
};

using Fields = std::vector<Field>;

void AddBool(Fields* fields, const std::string& name, bool* value) {
  fields->push_back(Field{
      name,
      [value](const std::string& text, std::string* error) {
        if (ParseBool(text, value)) {
          return true;
        }
        *error = "expected true or false";
        return false;
      },
      [value]() { return std::string(*value ? "True" : "False"); }});
}

void AddInt(Fields* fields, const std::string& name, int* value) {
  fields->push_back(Field{
      name,
      [value](const std::string& text, std::string* error) {
        if (ParseInt(text, value)) {
          return true;
        }
        *error = "expected an integer";
        return false;
      },
      [value]() { return std::to_string(*value); }});
}

void AddDouble(Fields* fields, const std::string& name, double* value) {
  fields->push_back(Field{
      name,
      [value](const std::string& text, std::string* error) {
        if (ParseDouble(text, value)) {
          return true;
        }
        *error = "expected a number";
        return false;
      },
      [value]() { return FormatDouble(*value); }});
}

// Enum values by name (case-insensitive) or by integer value.
template <typename E>
using Members = std::vector<std::pair<std::string, E>>;

template <typename E>
void AddEnum(Fields* fields,
             const std::string& name,
             E* value,
             const Members<E>& members) {
  fields->push_back(Field{
      name,
      [value, members](const std::string& text, std::string* error) {
        const std::string key = Upper(Trim(text));
        for (const auto& member : members) {
          if (Upper(member.first) == key) {
            *value = member.second;
            return true;
          }
        }
        int number = 0;
        if (ParseInt(text, &number)) {
          for (const auto& member : members) {
            if (static_cast<int>(member.second) == number) {
              *value = member.second;
              return true;
            }
          }
        }
        std::vector<std::string> names;
        for (const auto& member : members) {
          names.push_back(member.first);
        }
        *error = "expected one of " + Join(names, ", ");
        return false;
      },
      [value, members]() {
        for (const auto& member : members) {
          if (member.second == *value) {
            return member.first;
          }
        }
        return std::to_string(static_cast<int>(*value));
      }});
}

// Bit mask of OptimizeIntrinsicsType: "FOCAL_LENGTH|RADIAL_DISTORTION" (also
// "," as separator), a single convenience name, or an integer.
void AddIntrinsicsMask(Fields* fields,
                       const std::string& name,
                       theia::OptimizeIntrinsicsType* value) {
  using theia::OptimizeIntrinsicsType;
  static const Members<int> kFlags = {
      {"NONE", 0x00},
      {"FOCAL_LENGTH", 0x01},
      {"ASPECT_RATIO", 0x02},
      {"SKEW", 0x04},
      {"PRINCIPAL_POINTS", 0x08},
      {"RADIAL_DISTORTION", 0x10},
      {"TANGENTIAL_DISTORTION", 0x20},
      {"DISTORTION", 0x30},
      {"FOCAL_LENGTH_DISTORTION", 0x31},
      {"FOCAL_LENGTH_RADIAL_DISTORTION", 0x11},
      {"ALL", 0x3F},
  };
  fields->push_back(Field{
      name,
      [value](const std::string& text, std::string* error) {
        int mask = 0;
        std::string token;
        std::string spaced = text;
        std::replace(spaced.begin(), spaced.end(), ',', '|');
        std::istringstream stream(spaced);
        bool any = false;
        while (std::getline(stream, token, '|')) {
          token = Trim(token);
          if (token.empty()) {
            continue;
          }
          any = true;
          const std::string key = Upper(token);
          bool found = false;
          for (const auto& flag : kFlags) {
            if (flag.first == key) {
              mask |= flag.second;
              found = true;
              break;
            }
          }
          if (!found) {
            int number = 0;
            if (!ParseInt(token, &number) || number < 0 || number > 0x3F) {
              *error = "expected flag names joined by '|' (e.g. FOCAL_LENGTH|RADIAL_DISTORTION), "
                       "ALL, NONE or an integer in [0, 63]";
              return false;
            }
            mask |= number;
          }
        }
        if (!any) {
          mask = 0;
        }
        *value = static_cast<OptimizeIntrinsicsType>(mask);
        return true;
      },
      [value]() {
        const int bits = static_cast<int>(*value);
        for (const auto& named : kFlags) {
          if (named.second == bits) {
            return named.first;
          }
        }
        static const std::vector<std::pair<std::string, int>> kBasic = {
            {"FOCAL_LENGTH", 0x01},  {"ASPECT_RATIO", 0x02},
            {"SKEW", 0x04},          {"PRINCIPAL_POINTS", 0x08},
            {"RADIAL_DISTORTION", 0x10}, {"TANGENTIAL_DISTORTION", 0x20},
        };
        std::vector<std::string> parts;
        int covered = 0;
        for (const auto& flag : kBasic) {
          if (bits & flag.second) {
            parts.push_back(flag.first);
            covered |= flag.second;
          }
        }
        if (covered != bits) {
          return std::to_string(bits);
        }
        return Join(parts, "|");
      }});
}

// Builds the complete, ordered list of settable options. Every field refers
// into `o`, so the list is only valid while `o` is alive and not moved.
Fields BuildFields(ReconstructionEstimatorOptions* o) {
  using theia::GlobalPositionEstimatorType;
  using theia::GlobalRotationEstimatorType;
  using theia::LossFunctionType;
  using theia::PnPType;
  using theia::ReconstructionEstimatorType;
  using theia::TrackParametrizationType;
  using theia::TriangulationMethodType;

  Fields f;

  // Top level.
  AddEnum<ReconstructionEstimatorType>(
      &f, "reconstruction_estimator_type", &o->reconstruction_estimator_type,
      {{"GLOBAL", ReconstructionEstimatorType::GLOBAL},
       {"INCREMENTAL", ReconstructionEstimatorType::INCREMENTAL},
       {"HYBRID", ReconstructionEstimatorType::HYBRID}});
  AddEnum<GlobalRotationEstimatorType>(
      &f, "global_rotation_estimator_type", &o->global_rotation_estimator_type,
      {{"ROBUST_L1L2", GlobalRotationEstimatorType::ROBUST_L1L2},
       {"NONLINEAR", GlobalRotationEstimatorType::NONLINEAR},
       {"LINEAR", GlobalRotationEstimatorType::LINEAR},
       {"LAGRANGE_DUAL", GlobalRotationEstimatorType::LAGRANGE_DUAL},
       {"HYBRID", GlobalRotationEstimatorType::HYBRID}});
  AddEnum<GlobalPositionEstimatorType>(
      &f, "global_position_estimator_type", &o->global_position_estimator_type,
      {{"NONLINEAR", GlobalPositionEstimatorType::NONLINEAR},
       {"LINEAR_TRIPLET", GlobalPositionEstimatorType::LINEAR_TRIPLET},
       {"LEAST_UNSQUARED_DEVIATION", GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION},
       {"LIGT", GlobalPositionEstimatorType::LIGT},
       {"GLOMAP", GlobalPositionEstimatorType::GLOMAP},
       {"ART", GlobalPositionEstimatorType::ART}});
  AddInt(&f, "num_threads", &o->num_threads);
  AddDouble(&f, "max_reprojection_error_in_pixels", &o->max_reprojection_error_in_pixels);
  AddInt(&f, "min_num_two_view_inliers", &o->min_num_two_view_inliers);

  // RANSAC.
  AddDouble(&f, "ransac_confidence", &o->ransac_confidence);
  AddInt(&f, "ransac_min_iterations", &o->ransac_min_iterations);
  AddInt(&f, "ransac_max_iterations", &o->ransac_max_iterations);
  AddBool(&f, "ransac_use_mle", &o->ransac_use_mle);
  AddBool(&f, "ransac_use_lo", &o->ransac_use_lo);
  AddInt(&f, "ransac_lo_start_iterations", &o->ransac_lo_start_iterations);

  // Rotation and position filtering.
  AddDouble(&f, "rotation_filtering_max_difference_degrees",
            &o->rotation_filtering_max_difference_degrees);
  AddBool(&f, "refine_relative_translations_after_rotation_estimation",
          &o->refine_relative_translations_after_rotation_estimation);
  AddBool(&f, "refine_rotations_with_rotation_only_error",
          &o->refine_rotations_with_rotation_only_error);
  AddBool(&f, "extract_maximal_rigid_subgraph", &o->extract_maximal_rigid_subgraph);
  AddBool(&f, "filter_relative_translations_with_1dsfm",
          &o->filter_relative_translations_with_1dsfm);
  AddInt(&f, "translation_filtering_num_iterations",
         &o->translation_filtering_num_iterations);
  AddDouble(&f, "translation_filtering_projection_tolerance",
            &o->translation_filtering_projection_tolerance);

  // Global rotation.
  AddDouble(&f, "rotation_estimation_robust_loss_scale",
            &o->rotation_estimation_robust_loss_scale);

  // Global position: nested option groups.
  {
    const std::string p = "nonlinear_position_estimator_options.";
    auto& g = o->nonlinear_position_estimator_options;
    AddInt(&f, p + "num_threads", &g.num_threads);
    AddInt(&f, p + "max_num_iterations", &g.max_num_iterations);
    AddDouble(&f, p + "robust_loss_width", &g.robust_loss_width);
    AddInt(&f, p + "min_num_points_per_view", &g.min_num_points_per_view);
    AddDouble(&f, p + "point_to_camera_weight", &g.point_to_camera_weight);
  }
  {
    const std::string p = "linear_triplet_position_estimator_options.";
    auto& g = o->linear_triplet_position_estimator_options;
    AddInt(&f, p + "num_threads", &g.num_threads);
    AddInt(&f, p + "max_power_iterations", &g.max_power_iterations);
    AddDouble(&f, p + "eigensolver_threshold", &g.eigensolver_threshold);
  }
  {
    const std::string p = "ligt_position_estimator_options.";
    auto& g = o->ligt_position_estimator_options;
    AddInt(&f, p + "num_threads", &g.num_threads);
    AddInt(&f, p + "max_power_iterations", &g.max_power_iterations);
    AddDouble(&f, p + "eigensolver_threshold", &g.eigensolver_threshold);
    AddInt(&f, p + "max_num_views_svd", &g.max_num_views_svd);
    AddBool(&f, p + "normalize_constraints", &g.normalize_constraints);
    AddInt(&f, p + "num_irls_iterations", &g.num_irls_iterations);
    AddDouble(&f, p + "irls_scale_multiplier", &g.irls_scale_multiplier);
    AddDouble(&f, p + "irls_convergence_threshold", &g.irls_convergence_threshold);
    AddBool(&f, p + "use_scale_constraint", &g.use_scale_constraint);
    AddInt(&f, p + "min_tracks_per_view", &g.min_tracks_per_view);
  }
  {
    const std::string p = "art_position_estimator_options.";
    auto& g = o->art_position_estimator_options;
    AddInt(&f, p + "num_threads", &g.num_threads);
    AddInt(&f, p + "min_tracks_per_triplet", &g.min_tracks_per_triplet);
    AddInt(&f, p + "max_tracks_per_triplet", &g.max_tracks_per_triplet);
    AddInt(&f, p + "max_triplets_per_view", &g.max_triplets_per_view);
    AddInt(&f, p + "min_tracks_per_triplet_fallback",
           &g.min_tracks_per_triplet_fallback);
    AddInt(&f, p + "num_coverage_rounds", &g.num_coverage_rounds);
    AddInt(&f, p + "num_robust_samples", &g.num_robust_samples);
    AddInt(&f, p + "min_bridge_tracks", &g.min_bridge_tracks);
    AddBool(&f, p + "use_track_covisibility_triplets",
            &g.use_track_covisibility_triplets);
    AddDouble(&f, p + "max_eigen_ratio", &g.max_eigen_ratio);
    AddDouble(&f, p + "min_c_conditioning", &g.min_c_conditioning);
    AddInt(&f, p + "num_obs_filter_iterations", &g.num_obs_filter_iterations);
    AddDouble(&f, p + "obs_mad_factor", &g.obs_mad_factor);
    AddDouble(&f, p + "triplet_conf_factor", &g.triplet_conf_factor);
    AddInt(&f, p + "num_irls_iterations", &g.num_irls_iterations);
    AddDouble(&f, p + "irls_scale_multiplier", &g.irls_scale_multiplier);
    AddDouble(&f, p + "irls_scale", &g.irls_scale);
    AddDouble(&f, p + "irls_convergence_threshold", &g.irls_convergence_threshold);
  }
  {
    const std::string p = "rotation_only_refiner_options.";
    auto& g = o->rotation_only_refiner_options;
    AddInt(&f, p + "num_outer_iterations", &g.num_outer_iterations);
    AddInt(&f, p + "max_num_iterations", &g.max_num_iterations);
    AddDouble(&f, p + "loss_scale_px", &g.loss_scale_px);
    AddInt(&f, p + "max_observations_per_view", &g.max_observations_per_view);
    AddInt(&f, p + "max_tracks_per_triplet", &g.max_tracks_per_triplet);
    AddBool(&f, p + "use_two_view_tuples", &g.use_two_view_tuples);
    AddBool(&f, p + "require_view_graph_edges", &g.require_view_graph_edges);
    AddInt(&f, p + "max_reference_candidates", &g.max_reference_candidates);
    AddDouble(&f, p + "tuple_gate_px", &g.tuple_gate_px);
    AddDouble(&f, p + "verify_observations_px", &g.verify_observations_px);
  }
  {
    const std::string p = "glomap_position_estimator_options.";
    auto& g = o->glomap_position_estimator_options;
    AddInt(&f, p + "num_threads", &g.num_threads);
    AddInt(&f, p + "max_num_iterations", &g.max_num_iterations);
    AddDouble(&f, p + "robust_loss_width", &g.robust_loss_width);
    AddInt(&f, p + "min_track_length", &g.min_track_length);
    AddInt(&f, p + "max_num_tracks", &g.max_num_tracks);
    AddBool(&f, p + "write_points_to_reconstruction", &g.write_points_to_reconstruction);
    AddBool(&f, p + "use_pairwise_scale_priors", &g.use_pairwise_scale_priors);
    AddDouble(&f, p + "pairwise_scale_prior_weight", &g.pairwise_scale_prior_weight);
  }
  {
    const std::string p = "least_unsquared_deviation_position_estimator_options.";
    auto& g = o->least_unsquared_deviation_position_estimator_options;
    AddInt(&f, p + "max_num_iterations", &g.max_num_iterations);
    AddInt(&f, p + "max_num_reweighted_iterations", &g.max_num_reweighted_iterations);
    AddDouble(&f, p + "convergence_criterion", &g.convergence_criterion);
    AddBool(&f, p + "use_scale_estimates", &g.use_scale_estimates);
    AddDouble(&f, p + "min_valid_scale_estimate", &g.min_valid_scale_estimate);
  }

  AddBool(&f, "refine_camera_positions_and_points_after_position_estimation",
          &o->refine_camera_positions_and_points_after_position_estimation);

  // Incremental and hybrid SfM.
  AddDouble(&f, "multiple_view_localization_ratio", &o->multiple_view_localization_ratio);
  AddDouble(&f, "absolute_pose_reprojection_error_threshold",
            &o->absolute_pose_reprojection_error_threshold);
  AddInt(&f, "min_num_absolute_pose_inliers", &o->min_num_absolute_pose_inliers);
  AddDouble(&f, "full_bundle_adjustment_growth_percent",
            &o->full_bundle_adjustment_growth_percent);
  AddInt(&f, "partial_bundle_adjustment_num_views", &o->partial_bundle_adjustment_num_views);
  AddDouble(&f, "relative_position_estimation_max_sampson_error_pixels",
            &o->relative_position_estimation_max_sampson_error_pixels);

  // Triangulation.
  AddDouble(&f, "min_triangulation_angle_degrees", &o->min_triangulation_angle_degrees);
  AddDouble(&f, "triangulation_max_reprojection_error_in_pixels",
            &o->triangulation_max_reprojection_error_in_pixels);
  AddBool(&f, "bundle_adjust_tracks", &o->bundle_adjust_tracks);
  AddEnum<TriangulationMethodType>(
      &f, "triangulation_method", &o->triangulation_method,
      {{"MIDPOINT", TriangulationMethodType::MIDPOINT},
       {"SVD", TriangulationMethodType::SVD},
       {"L2_MINIMIZATION", TriangulationMethodType::L2_MINIMIZATION}});
  AddInt(&f, "num_retriangulation_iterations", &o->num_retriangulation_iterations);

  // Bundle adjustment.
  AddEnum<LossFunctionType>(
      &f, "bundle_adjustment_loss_function_type", &o->bundle_adjustment_loss_function_type,
      {{"TRIVIAL", LossFunctionType::TRIVIAL},
       {"HUBER", LossFunctionType::HUBER},
       {"SOFTLONE", LossFunctionType::SOFTLONE},
       {"CAUCHY", LossFunctionType::CAUCHY},
       {"ARCTAN", LossFunctionType::ARCTAN},
       {"TUKEY", LossFunctionType::TUKEY},
       {"TRUNCATED", LossFunctionType::TRUNCATED}});
  AddDouble(&f, "bundle_adjustment_robust_loss_width", &o->bundle_adjustment_robust_loss_width);
  AddInt(&f, "min_cameras_for_iterative_solver", &o->min_cameras_for_iterative_solver);
  AddIntrinsicsMask(&f, "intrinsics_to_optimize", &o->intrinsics_to_optimize);
  AddEnum<TrackParametrizationType>(
      &f, "track_parametrization_type", &o->track_parametrization_type,
      {{"XYZW", TrackParametrizationType::XYZW},
       {"XYZW_MANIFOLD", TrackParametrizationType::XYZW_MANIFOLD},
       {"INVERSE_DEPTH", TrackParametrizationType::INVERSE_DEPTH}});

  // Ceres solver selection. Ceres enums are unscoped, so their members are
  // referenced as ceres::NAME.
  AddEnum<ceres::DenseLinearAlgebraLibraryType>(
      &f, "dense_linear_algebra_library_type", &o->dense_linear_algebra_library_type,
      {{"EIGEN", ceres::EIGEN}, {"LAPACK", ceres::LAPACK}, {"CUDA", ceres::CUDA}});
  AddEnum<ceres::SparseLinearAlgebraLibraryType>(
      &f, "sparse_linear_algebra_library_type", &o->sparse_linear_algebra_library_type,
      {{"SUITE_SPARSE", ceres::SUITE_SPARSE},
       {"EIGEN_SPARSE", ceres::EIGEN_SPARSE},
       {"ACCELERATE_SPARSE", ceres::ACCELERATE_SPARSE},
       {"CUDA_SPARSE", ceres::CUDA_SPARSE},
       {"NO_SPARSE", ceres::NO_SPARSE}});
  AddEnum<ceres::LinearSolverType>(
      &f, "linear_solver_type", &o->linear_solver_type,
      {{"DENSE_NORMAL_CHOLESKY", ceres::DENSE_NORMAL_CHOLESKY},
       {"DENSE_QR", ceres::DENSE_QR},
       {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY},
       {"DENSE_SCHUR", ceres::DENSE_SCHUR},
       {"SPARSE_SCHUR", ceres::SPARSE_SCHUR},
       {"ITERATIVE_SCHUR", ceres::ITERATIVE_SCHUR},
       {"CGNR", ceres::CGNR}});
  AddEnum<ceres::PreconditionerType>(
      &f, "preconditioner_type", &o->preconditioner_type,
      {{"IDENTITY", ceres::IDENTITY},
       {"JACOBI", ceres::JACOBI},
       {"SCHUR_JACOBI", ceres::SCHUR_JACOBI},
       {"SCHUR_POWER_SERIES_EXPANSION", ceres::SCHUR_POWER_SERIES_EXPANSION},
       {"CLUSTER_JACOBI", ceres::CLUSTER_JACOBI},
       {"CLUSTER_TRIDIAGONAL", ceres::CLUSTER_TRIDIAGONAL},
       {"SUBSET", ceres::SUBSET}});
  AddEnum<ceres::VisibilityClusteringType>(
      &f, "visibility_clustering_type", &o->visibility_clustering_type,
      {{"CANONICAL_VIEWS", ceres::CANONICAL_VIEWS},
       {"SINGLE_LINKAGE", ceres::SINGLE_LINKAGE}});
  AddInt(&f, "max_num_iterations", &o->max_num_iterations);
  AddInt(&f, "use_inner_iterations", &o->use_inner_iterations);  // int in the header
  AddBool(&f, "optimize_for_forward_facing_trajectory",
          &o->optimize_for_forward_facing_trajectory);

  // Track subsampling.
  AddBool(&f, "subsample_tracks_for_bundle_adjustment",
          &o->subsample_tracks_for_bundle_adjustment);
  AddInt(&f, "track_subset_selection_long_track_length_threshold",
         &o->track_subset_selection_long_track_length_threshold);
  AddInt(&f, "track_selection_image_grid_cell_size_pixels",
         &o->track_selection_image_grid_cell_size_pixels);
  AddInt(&f, "min_num_optimized_tracks_per_view", &o->min_num_optimized_tracks_per_view);

  // Localization.
  AddEnum<PnPType>(&f, "localization_pnp_type", &o->localization_pnp_type,
                   {{"KNEIP", PnPType::KNEIP},
                    {"DLS", PnPType::DLS},
                    {"SQPnP", PnPType::SQPnP},
                    {"MLPnP", PnPType::MLPnP}});

  return f;
}

const Field* FindField(const Fields& fields, const std::string& name) {
  for (const auto& field : fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

bool IsGroupPrefix(const Fields& fields, const std::string& name) {
  const std::string prefix = name + ".";
  for (const auto& field : fields) {
    if (field.name.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

// Applies a parsed JSON object. Nested objects are option groups.
bool ApplyJsonObject(const rapidjson::Value& object,
                     const std::string& prefix,
                     const Fields& fields,
                     std::string* error) {
  for (auto it = object.MemberBegin(); it != object.MemberEnd(); ++it) {
    const std::string key = prefix + it->name.GetString();
    const rapidjson::Value& value = it->value;

    if (value.IsObject()) {
      if (!IsGroupPrefix(fields, key)) {
        *error = FindField(fields, key) != nullptr
                     ? "option '" + key + "' is not a group of options"
                     : "unknown option group '" + key + "'";
        return false;
      }
      if (!ApplyJsonObject(value, key + ".", fields, error)) {
        return false;
      }
      continue;
    }

    const Field* field = FindField(fields, key);
    if (field == nullptr) {
      *error = IsGroupPrefix(fields, key)
                   ? "option '" + key + "' is a group, not a value"
                   : "unknown option '" + key + "'";
      return false;
    }

    std::string text;
    if (value.IsString()) {
      text = value.GetString();
    } else if (value.IsBool()) {
      text = value.GetBool() ? "true" : "false";
    } else if (value.IsInt64()) {
      text = std::to_string(value.GetInt64());
    } else if (value.IsUint64()) {
      text = std::to_string(value.GetUint64());
    } else if (value.IsDouble()) {
      char buffer[64];
      std::snprintf(buffer, sizeof(buffer), "%.17g", value.GetDouble());
      text = buffer;
    } else {
      *error = "option '" + key + "' must be a number, boolean, string or enum name";
      return false;
    }

    std::string message;
    if (!field->parse(text, &message)) {
      *error = "invalid value for '" + key + "': " + text + " (" + message + ")";
      return false;
    }
  }
  return true;
}

}  // namespace

bool ApplyOptionsJsonFile(const std::string& path,
                          ReconstructionEstimatorOptions* options,
                          std::string* error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    *error = "cannot read config file " + path;
    return false;
  }
  std::stringstream buffer;
  buffer << stream.rdbuf();

  rapidjson::Document document;
  document.Parse(buffer.str().c_str());
  if (document.HasParseError()) {
    *error = "config file " + path + " is not valid JSON: " +
             rapidjson::GetParseError_En(document.GetParseError()) + " at offset " +
             std::to_string(document.GetErrorOffset());
    return false;
  }
  if (!document.IsObject()) {
    *error = "config file " + path + " must contain a JSON object";
    return false;
  }
  const Fields fields = BuildFields(options);
  return ApplyJsonObject(document, "", fields, error);
}

bool ApplyOptionSetting(const std::string& setting,
                        ReconstructionEstimatorOptions* options,
                        std::string* error) {
  const size_t equals = setting.find('=');
  if (equals == std::string::npos) {
    *error = "--set expects key=value, got '" + setting + "'";
    return false;
  }
  const std::string key = Trim(setting.substr(0, equals));
  const std::string value = setting.substr(equals + 1);
  if (key.empty()) {
    *error = "--set has an empty key in '" + setting + "'";
    return false;
  }
  const Fields fields = BuildFields(options);
  const Field* field = FindField(fields, key);
  if (field == nullptr) {
    *error = IsGroupPrefix(fields, key)
                 ? "option '" + key + "' is a group; set one of its fields"
                 : "unknown option '" + key + "'";
    return false;
  }
  std::string message;
  if (!field->parse(value, &message)) {
    *error = "invalid value for '" + key + "': " + value + " (" + message + ")";
    return false;
  }
  return true;
}

std::vector<std::string> ListOptions(const ReconstructionEstimatorOptions& options) {
  ReconstructionEstimatorOptions copy = options;
  Fields fields = BuildFields(&copy);
  std::sort(fields.begin(), fields.end(),
            [](const Field& a, const Field& b) { return a.name < b.name; });
  std::vector<std::string> lines;
  lines.reserve(fields.size());
  for (const auto& field : fields) {
    lines.push_back(field.name + " = " + field.format());
  }
  return lines;
}

std::vector<char*> ExtractRepeatedFlag(const std::string& name,
                                       int argc,
                                       char** argv,
                                       std::vector<std::string>* values) {
  std::vector<char*> kept;
  if (argc > 0) {
    kept.push_back(argv[0]);
  }
  const std::string flag = "--" + name;
  const std::string flag_eq = flag + "=";
  bool stop = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (!stop && arg == "--") {
      stop = true;
    } else if (!stop && arg == flag) {
      // A missing value is reported by the caller as an invalid setting.
      values->push_back(i + 1 < argc ? std::string(argv[++i]) : std::string());
      continue;
    } else if (!stop && arg.compare(0, flag_eq.size(), flag_eq) == 0) {
      values->push_back(arg.substr(flag_eq.size()));
      continue;
    }
    kept.push_back(argv[i]);
  }
  return kept;
}

}  // namespace theia_apps
