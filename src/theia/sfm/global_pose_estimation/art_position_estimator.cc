// Implements ART (Analytical Rotation-guided Translation), Sec. 5.3 / Alg. 3 of
// Li, Cai, Wu, arXiv:2611.11770. Parameters not given in the paper are pyTheia
// choices. See art_position_estimator.h for the details and the citations.

#include "theia/sfm/global_pose_estimation/art_position_estimator.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "theia/sfm/global_pose_estimation/ligt_constraints.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/util/map_util.h"
#include "theia/util/parallel_for.h"

namespace theia {

namespace {

using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6x3d = Eigen::Matrix<double, 6, 3>;

// Smallest allowed value for denominators that are scales (|alpha|, norms).
constexpr double kEps = 1e-12;
// Floor for the observation-filter threshold, so that noise-free residuals
// (~1e-16) do not reject tracks.
constexpr double kTinyResidual = 1e-10;
// Floor for the triplet confidence threshold tau_r (noise-free data has a
// median confidence of ~0).
constexpr double kTinyConfidence = 1e-9;
// Conditioning of C^T C used by the public SolveARTTriplet.
constexpr double kDefaultMinConditioning = 1e-10;
// Absolute scale floor of the blocks, per unit track weight. The blocks are
// built from normalized rays and vanish exactly for degenerate triplets (e.g.
// identical camera centers), where relative checks alone would pass.
constexpr double kMinBlockScale = 1e-18;

Matrix3d AngleAxisToRotationMatrix(const Vector3d& angle_axis) {
  Matrix3d rotation_matrix;
  ceres::AngleAxisToRotationMatrix(angle_axis.data(), rotation_matrix.data());
  return rotation_matrix;
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  double median = values[mid];
  if (values.size() % 2 == 0) {
    const double lower = *std::max_element(values.begin(), values.begin() + mid);
    median = 0.5 * (median + lower);
  }
  return median;
}

// Blocks of Theorem 1 for one track: B_k and C_k (6x3 each). Rows 0-2 come
// from ComputeLiGTBC(eta, nu, zeta), rows 3-5 from ComputeLiGTBC(zeta, nu, eta),
// so that B_k e + C_k b_en = 0 for the true baselines (e = b_ez, b_en).
void ComputeARTBlocks(const Vector3d& x_eta,
                      const Vector3d& x_zeta,
                      const Vector3d& x_nu,
                      const Matrix3d& R_eta,
                      const Matrix3d& R_zeta,
                      const Matrix3d& R_nu,
                      Matrix6x3d* B_k,
                      Matrix6x3d* C_k) {
  Matrix3d B1, C1, B2, C2;
  ComputeLiGTBC(x_eta, x_nu, x_zeta, R_eta, R_nu, R_zeta, &B1, &C1);
  ComputeLiGTBC(x_zeta, x_nu, x_eta, R_zeta, R_nu, R_eta, &B2, &C2);
  B_k->topRows<3>() = B1;
  B_k->bottomRows<3>() = -(B2 + C2);
  C_k->topRows<3>() = C1;
  C_k->bottomRows<3>() = C2;
}

// Number of tracks whose depths are both positive for the edge (a, o), where
// b = c_a - c_o is the world baseline. With X_o = R_o (X - c_o) and
// X_a = R_a (X - c_a), a depth-positive point satisfies
//   z_o x_o = z_a R_oa x_a + R_o b,  R_oa = R_o R_a^T,
// which is solved for (z_a, z_o) by least squares.
int CountPositiveDepths(const std::vector<Vector3d>& x_a,
                        const std::vector<Vector3d>& x_o,
                        const Matrix3d& R_a,
                        const Matrix3d& R_o,
                        const Vector3d& b) {
  const Matrix3d R_oa = R_o * R_a.transpose();
  const Vector3d t = R_o * b;
  int count = 0;
  for (size_t k = 0; k < x_a.size(); ++k) {
    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = -(R_oa * x_a[k]);
    A.col(1) = x_o[k];
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(
        A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::Vector2d z = svd.solve(t);
    if (z(0) > 0.0 && z(1) > 0.0) {
      ++count;
    }
  }
  return count;
}

// Theorem 1 from precomputed blocks. Fills b_ez (unit norm, sign not yet
// fixed), b_en, T, eigenvalues and eigen_ratio.
bool SolveARTFromBlocks(const std::vector<Matrix6x3d>& B_k,
                        const std::vector<Matrix6x3d>& C_k,
                        const std::vector<double>& weights,
                        const double min_conditioning,
                        ARTTripletSolution* sol) {
  Matrix3d CtC = Matrix3d::Zero();
  Matrix3d CtB = Matrix3d::Zero();
  Matrix3d BtB = Matrix3d::Zero();
  double sum_w = 0.0;
  for (size_t k = 0; k < B_k.size(); ++k) {
    const double w = weights[k];
    sum_w += w;
    CtC += w * C_k[k].transpose() * C_k[k];
    CtB += w * C_k[k].transpose() * B_k[k];
    BtB += w * B_k[k].transpose() * B_k[k];
  }
  if (!(sum_w > 0.0)) {
    return false;
  }
  const double min_scale = kMinBlockScale * sum_w;

  // Rank check of C^T C (relative smallest eigenvalue).
  Eigen::SelfAdjointEigenSolver<Matrix3d> eig_c(CtC);
  if (eig_c.info() != Eigen::Success) {
    return false;
  }
  const double lambda_max = eig_c.eigenvalues().maxCoeff();
  const double lambda_min = eig_c.eigenvalues().minCoeff();
  if (!(lambda_max > min_scale) ||
      !(lambda_min >= min_conditioning * lambda_max)) {
    return false;
  }

  Eigen::LDLT<Matrix3d> ldlt(CtC);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }
  const Matrix3d T = ldlt.solve(CtB);

  Matrix3d S = BtB - CtB.transpose() * T;
  S = 0.5 * (S + S.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix3d> eig(S);
  if (eig.info() != Eigen::Success) {
    return false;
  }
  const Vector3d values = eig.eigenvalues();  // ascending
  // Degenerate configuration (e.g. identical camera centers): S vanishes, so
  // the direction e is not determined. Reject if the second eigenvalue is
  // numerically zero relative to the scale of the problem.
  const double scale = BtB.trace();
  if (!(scale > min_scale) || !(values(1) > 1e-10 * scale)) {
    return false;
  }
  const Vector3d e = eig.eigenvectors().col(0);
  sol->b_ez = e;
  sol->b_en = -(T * e);
  sol->T = T;
  sol->eigenvalues = values;
  sol->eigen_ratio = values(0) / values(1);
  return sol->b_ez.allFinite() && sol->b_en.allFinite() &&
         std::isfinite(sol->eigen_ratio);
}

// Theorem 1 including the chirality fix (sign of the solution).
bool SolveARTWithBlocks(const std::vector<Vector3d>& x_eta,
                        const std::vector<Vector3d>& x_zeta,
                        const std::vector<Vector3d>& x_nu,
                        const Matrix3d& R_eta,
                        const Matrix3d& R_zeta,
                        const Matrix3d& R_nu,
                        const std::vector<Matrix6x3d>& B_k,
                        const std::vector<Matrix6x3d>& C_k,
                        const std::vector<double>& weights,
                        const double min_conditioning,
                        ARTTripletSolution* sol) {
  if (!SolveARTFromBlocks(B_k, C_k, weights, min_conditioning, sol)) {
    return false;
  }
  // Step 7: chirality. Count the depth-positive tracks on both edges and
  // negate the solution if fewer than half of the 2K checks pass.
  const int pass_ez =
      CountPositiveDepths(x_eta, x_zeta, R_eta, R_zeta, sol->b_ez);
  const int pass_en = CountPositiveDepths(x_eta, x_nu, R_eta, R_nu, sol->b_en);
  if (pass_ez + pass_en < static_cast<int>(x_eta.size())) {
    sol->b_ez = -sol->b_ez;
    sol->b_en = -sol->b_en;
  }
  return true;
}

// Result of one reference-edge choice of a triplet.
struct ReferenceChoice {
  ARTTripletSolution solution;
  double conf = 0.0;
  // Indices (ascending) of the tracks that survive the observation filter.
  std::vector<int> kept;
};

// Solves one reference-edge choice with observation filtering (Alg. 3 step 2).
// rays[v][k] is the normalized ray of track k in view v (v = 0, 1, 2 for the
// sorted triplet views), R[v] its rotation. (eta, zeta, nu) index these views.
bool SolveReferenceChoice(const std::array<std::vector<Vector3d>, 3>& rays,
                          const std::array<Matrix3d, 3>& R,
                          const int eta,
                          const int zeta,
                          const int nu,
                          const ARTPositionEstimator::Options& opt,
                          ReferenceChoice* choice) {
  const int num_tracks = static_cast<int>(rays[0].size());
  std::vector<int> keep(num_tracks);
  std::iota(keep.begin(), keep.end(), 0);

  // Robust start (least median of squares). The first filter pass measures
  // the residuals against a least-squares solution, which gross outliers
  // (e.g. random mismatches in about a quarter of the tracks) can ruin, and
  // then the filter keeps the wrong tracks. So the starting solution is the
  // one with the lowest median residual among the least-squares solution and
  // solutions from random subsets of kRobustSampleSize tracks, if it is
  // clearly better than the least-squares one; the first pass then keeps the
  // tracks within the MAD threshold of it. Fixed seed: the result
  // does not depend on the call order or thread.
  constexpr int kRobustSampleSize = 8;
  constexpr double kRobustStartGain = 0.5;
  if (opt.num_robust_samples > 0 && opt.num_obs_filter_iterations > 0 &&
      num_tracks >= 2 * kRobustSampleSize) {
    std::vector<Matrix6x3d> B_all(num_tracks), C_all(num_tracks);
    for (int k = 0; k < num_tracks; ++k) {
      ComputeARTBlocks(rays[eta][k], rays[zeta][k], rays[nu][k], R[eta],
                       R[zeta], R[nu], &B_all[k], &C_all[k]);
    }
    auto residuals = [&](const ARTTripletSolution& sol) {
      std::vector<double> res(num_tracks);
      for (int k = 0; k < num_tracks; ++k) {
        const Vector6d r = B_all[k] * sol.b_ez + C_all[k] * sol.b_en;
        res[k] = r.norm() / (B_all[k].norm() + C_all[k].norm() + kEps);
      }
      return res;
    };
    auto solve_subset = [&](const std::vector<int>& subset,
                            ARTTripletSolution* sol) {
      const size_t n = subset.size();
      std::vector<Vector3d> a(n), b(n), c(n);
      std::vector<Matrix6x3d> Bs(n), Cs(n);
      for (size_t i = 0; i < n; ++i) {
        a[i] = rays[eta][subset[i]];
        b[i] = rays[zeta][subset[i]];
        c[i] = rays[nu][subset[i]];
        Bs[i] = B_all[subset[i]];
        Cs[i] = C_all[subset[i]];
      }
      return SolveARTWithBlocks(a, b, c, R[eta], R[zeta], R[nu], Bs, Cs,
                                std::vector<double>(n, 1.0),
                                opt.min_c_conditioning, sol);
    };
    double best_median = std::numeric_limits<double>::infinity();
    std::vector<double> best_res;
    ARTTripletSolution sol;
    double ls_median = std::numeric_limits<double>::infinity();
    if (solve_subset(keep, &sol)) {
      ls_median = Median(residuals(sol));
    }
    std::mt19937 rng(static_cast<unsigned int>(num_tracks) * 7919u + 17u);
    std::vector<int> pool = keep;
    for (int sample = 0; sample < opt.num_robust_samples; ++sample) {
      for (int i = 0; i < kRobustSampleSize; ++i) {
        std::uniform_int_distribution<int> pick(i, num_tracks - 1);
        std::swap(pool[i], pool[pick(rng)]);
      }
      const std::vector<int> subset(pool.begin(),
                                    pool.begin() + kRobustSampleSize);
      if (!solve_subset(subset, &sol)) {
        continue;
      }
      std::vector<double> res = residuals(sol);
      const double median = Median(res);
      if (median < best_median) {
        best_median = median;
        best_res = std::move(res);
      }
    }
    // Only a clearly better subset solution replaces the least-squares start
    // (the median residual drops by half or more, a sign of gross outliers);
    // on clean data the filter then works exactly as without this step.
    if (!best_res.empty() && best_median < kRobustStartGain * ls_median) {
      std::vector<double> deviations(num_tracks);
      for (int k = 0; k < num_tracks; ++k) {
        deviations[k] = std::abs(best_res[k] - best_median);
      }
      const double threshold = std::max(
          opt.obs_mad_factor * 1.4826 * Median(deviations), kTinyResidual);
      std::vector<int> next;
      for (int k = 0; k < num_tracks; ++k) {
        if (best_res[k] <= best_median + threshold) {
          next.push_back(k);
        }
      }
      keep.swap(next);
    }
  }

  for (int pass = 0;; ++pass) {
    if (static_cast<int>(keep.size()) < opt.min_tracks_per_triplet) {
      return false;
    }
    const size_t m = keep.size();
    std::vector<Vector3d> x_eta(m), x_zeta(m), x_nu(m);
    std::vector<Matrix6x3d> B_k(m), C_k(m);
    for (size_t i = 0; i < m; ++i) {
      const int k = keep[i];
      x_eta[i] = rays[eta][k];
      x_zeta[i] = rays[zeta][k];
      x_nu[i] = rays[nu][k];
      ComputeARTBlocks(x_eta[i], x_zeta[i], x_nu[i], R[eta], R[zeta], R[nu],
                       &B_k[i], &C_k[i]);
    }
    const std::vector<double> weights(m, 1.0);
    if (!SolveARTWithBlocks(x_eta, x_zeta, x_nu, R[eta], R[zeta], R[nu], B_k,
                            C_k, weights, opt.min_c_conditioning,
                            &choice->solution)) {
      return false;
    }

    // Normalized residuals r_k = ||B_k e + C_k b_en|| / (||B_k|| + ||C_k||).
    std::vector<double> s(m);
    for (size_t i = 0; i < m; ++i) {
      Vector6d r = B_k[i] * choice->solution.b_ez + C_k[i] * choice->solution.b_en;
      s[i] = r.norm() / (B_k[i].norm() + C_k[i].norm() + kEps);
    }
    const double median = Median(s);
    if (pass >= opt.num_obs_filter_iterations) {
      choice->conf = median;
      choice->kept = keep;
      return true;
    }

    std::vector<double> deviations(m);
    for (size_t i = 0; i < m; ++i) {
      deviations[i] = std::abs(s[i] - median);
    }
    const double mad = Median(deviations);
    const double threshold =
        std::max(opt.obs_mad_factor * 1.4826 * mad, kTinyResidual);
    std::vector<int> next;
    next.reserve(m);
    for (size_t i = 0; i < m; ++i) {
      if (s[i] <= threshold) {
        next.push_back(keep[i]);
      }
    }
    if (next.size() == m) {
      choice->conf = median;
      choice->kept = keep;
      return true;
    }
    keep.swap(next);
  }
}

// A valid triplet after the per-triplet solve (reference edge (eta, zeta)).
struct ARTTriplet {
  ViewId eta = kInvalidViewId;
  ViewId zeta = kInvalidViewId;
  ViewId nu = kInvalidViewId;
  Vector3d b_ez = Vector3d::Zero();
  Vector3d b_en = Vector3d::Zero();
  double conf = 0.0;
  double eigen_ratio = 0.0;
};

std::array<ViewId, 3> TripletViews(const ARTTriplet& t) {
  return {t.eta, t.zeta, t.nu};
}

ViewIdPair MakeEdge(const ViewId a, const ViewId b) {
  return a < b ? ViewIdPair(a, b) : ViewIdPair(b, a);
}

std::array<ViewIdPair, 3> TripletEdges(const ARTTriplet& t) {
  return {MakeEdge(t.eta, t.zeta), MakeEdge(t.eta, t.nu),
          MakeEdge(t.zeta, t.nu)};
}

// Union-find over the edges of a set of triplets. Two triplets are linked iff
// they share an edge, so the components are the edge-connected components.
class EdgeComponents {
 public:
  void AddTriplet(const ARTTriplet& t) {
    const auto edges = TripletEdges(t);
    const int a = EnsureEdge(edges[0]);
    const int b = EnsureEdge(edges[1]);
    const int c = EnsureEdge(edges[2]);
    Unite(a, b);
    Unite(a, c);
  }

  // Id of an edge of an added triplet, or -1 if the edge is not part of any.
  int FindEdge(const ViewIdPair& edge) const {
    const auto it = ids_.find(edge);
    return it == ids_.end() ? -1 : it->second;
  }

  // Root of the component of a triplet (any of its edges is a valid key).
  int RootOfTriplet(const ARTTriplet& t) {
    return Find(FindEdge(TripletEdges(t)[0]));
  }

  int Root(const int id) { return Find(id); }

 private:
  int EnsureEdge(const ViewIdPair& edge) {
    const auto it = ids_.find(edge);
    if (it != ids_.end()) {
      return it->second;
    }
    const int id = static_cast<int>(parent_.size());
    parent_.push_back(id);
    ids_[edge] = id;
    return id;
  }

  int Find(int x) {
    while (parent_[x] != x) {
      parent_[x] = parent_[parent_[x]];
      x = parent_[x];
    }
    return x;
  }

  void Unite(const int a, const int b) {
    const int ra = Find(a);
    const int rb = Find(b);
    if (ra != rb) {
      parent_[rb] = ra;
    }
  }

  std::map<ViewIdPair, int> ids_;
  std::vector<int> parent_;
};

// Components of triplets whose positions are fixed relative to each other up
// to one common scale and translation (rotations are known). A triplet fixes
// its three centers up to scale and translation. Triplets that share an edge
// are linked, and two components that share at least two views are merged:
// the two common centers fix the relative scale and translation. The second
// rule also covers clusters such as {(a, b, x), (a, b, y)} with x and y in the
// main component, which edge-connectivity alone would drop. Returns a
// component id per entry of `indices`; the result is deterministic.
std::vector<int> RigidComponents(const std::vector<ARTTriplet>& triplets,
                                 const std::vector<int>& indices) {
  const int n = static_cast<int>(indices.size());
  std::vector<int> parent(n);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&parent](int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };
  auto unite = [&](const int a, const int b) {
    const int ra = find(a);
    const int rb = find(b);
    if (ra == rb) {
      return false;
    }
    if (ra < rb) {
      parent[rb] = ra;
    } else {
      parent[ra] = rb;
    }
    return true;
  };
  std::map<ViewIdPair, int> edge_owner;
  for (int i = 0; i < n; ++i) {
    for (const ViewIdPair& e : TripletEdges(triplets[indices[i]])) {
      const auto inserted = edge_owner.emplace(e, i);
      if (!inserted.second) {
        unite(i, inserted.first->second);
      }
    }
  }
  bool changed = true;
  while (changed) {
    changed = false;
    std::map<int, std::set<ViewId>> component_views;
    for (int i = 0; i < n; ++i) {
      for (const ViewId v : TripletViews(triplets[indices[i]])) {
        component_views[find(i)].insert(v);
      }
    }
    std::map<ViewId, std::vector<int>> components_of_view;
    for (const auto& entry : component_views) {
      for (const ViewId v : entry.second) {
        components_of_view[v].push_back(entry.first);
      }
    }
    for (const auto& entry : component_views) {
      std::map<int, int> num_shared;
      for (const ViewId v : entry.second) {
        for (const int other : components_of_view[v]) {
          if (other != entry.first && ++num_shared[other] == 2) {
            changed |= unite(entry.first, other);
          }
        }
      }
    }
  }
  std::vector<int> component(n);
  for (int i = 0; i < n; ++i) {
    component[i] = find(i);
  }
  return component;
}

// The triplets of `candidates` (indices into `triplets`) that are rigidly
// connected to `seed` (see RigidComponents).
std::vector<int> EdgeConnectedComponent(const std::vector<ARTTriplet>& triplets,
                                        const std::vector<int>& candidates,
                                        const int seed) {
  const std::vector<int> ids = RigidComponents(triplets, candidates);
  int seed_id = -1;
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (candidates[i] == seed) {
      seed_id = ids[i];
    }
  }
  std::vector<int> component;
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (ids[i] == seed_id) {
      component.push_back(candidates[i]);
    }
  }
  return component;
}

// A LiGT track constraint (Cai et al. 2023, Eq. 8) on three views of a track:
// (B + C) c_i - B c_l - C c_j = 0, divided by the norm of its blocks. It is
// linear in the centers with no scale unknown; ART uses it to attach views
// that its triplets cannot attach (see EstimatePositions).
struct TrackLink {
  ViewId i = kInvalidViewId;
  ViewId j = kInvalidViewId;
  ViewId l = kInvalidViewId;
  Matrix3d B = Matrix3d::Zero();
  Matrix3d C = Matrix3d::Zero();
};

// The elements of the global system: triplets (with a scale alpha) and track
// links (without). group is -1 for the main rigid component and otherwise the
// id of a body attached to it.
struct GlobalProblem {
  std::vector<int> triplets;  // indices into the triplet list
  std::vector<int> triplet_group;
  std::vector<TrackLink> links;
  std::vector<int> link_group;

  int NumTriplets() const { return static_cast<int>(triplets.size()); }
  int NumElements() const {
    return static_cast<int>(triplets.size() + links.size());
  }
};

// Layout of the global linear system (Eq. 24-26, plus the track links).
// Unknowns: the positions of all views except the reference view (3 each),
// then alpha for every triplet except the seed (alpha_seed = 1).
struct GlobalLayout {
  std::vector<int> alpha_col;              // per triplet, -1 for the seed
  std::unordered_map<ViewId, int> block;   // view -> first position column
  int num_vars = 0;
};

GlobalLayout MakeLayout(const std::vector<ARTTriplet>& triplets,
                        const GlobalProblem& problem,
                        const int seed,
                        const ViewId reference) {
  GlobalLayout layout;
  std::set<ViewId> views;
  for (const int t : problem.triplets) {
    for (const ViewId v : TripletViews(triplets[t])) {
      views.insert(v);
    }
  }
  for (const TrackLink& link : problem.links) {
    views.insert(link.i);
    views.insert(link.j);
    views.insert(link.l);
  }
  views.erase(reference);
  int next = 0;
  for (const ViewId v : views) {
    layout.block[v] = next;
    next += 3;
  }
  for (const int t : problem.triplets) {
    layout.alpha_col.push_back(t == seed ? -1 : next++);
  }
  layout.num_vars = next;
  return layout;
}

// Position column block of a view, or -1 for the reference view (c = 0).
int BlockOf(const GlobalLayout& layout, const ViewId view) {
  const auto it = layout.block.find(view);
  return it == layout.block.end() ? -1 : it->second;
}

Vector3d PositionOf(const GlobalLayout& layout,
                    const Eigen::VectorXd& x,
                    const ViewId view) {
  const int block = BlockOf(layout, view);
  return block < 0 ? Vector3d::Zero() : Vector3d(x.segment<3>(block));
}

double AlphaOf(const GlobalLayout& layout, const Eigen::VectorXd& x,
               const int i) {
  return layout.alpha_col[i] < 0 ? 1.0 : x(layout.alpha_col[i]);
}

// Residual norm of triplet i: c_eta - c_zeta - alpha b_ez and
// c_eta - c_nu - alpha b_en.
double TripletResidualNorm(const std::vector<ARTTriplet>& triplets,
                           const GlobalProblem& problem,
                           const GlobalLayout& layout,
                           const Eigen::VectorXd& x,
                           const int i) {
  const ARTTriplet& t = triplets[problem.triplets[i]];
  const Vector3d c_eta = PositionOf(layout, x, t.eta);
  const double alpha = AlphaOf(layout, x, i);
  Vector6d r;
  r.head<3>() = (c_eta - PositionOf(layout, x, t.zeta)) - alpha * t.b_ez;
  r.tail<3>() = (c_eta - PositionOf(layout, x, t.nu)) - alpha * t.b_en;
  return r.norm();
}

double LinkResidualNorm(const TrackLink& link, const GlobalLayout& layout,
                        const Eigen::VectorXd& x) {
  return ((link.B + link.C) * PositionOf(layout, x, link.i) -
          link.B * PositionOf(layout, x, link.l) -
          link.C * PositionOf(layout, x, link.j))
      .norm();
}

bool SolveSymmetricPositiveDefinite(const Eigen::SparseMatrix<double>& A,
                                    const Eigen::VectorXd& b,
                                    Eigen::VectorXd* x) {
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(A);
  if (ldlt.info() == Eigen::Success) {
    *x = ldlt.solve(b);
    if (ldlt.info() == Eigen::Success && x->allFinite()) {
      return true;
    }
  }
  // Tiny Tikhonov term, only used when the factorization fails.
  const double trace = A.diagonal().sum();
  const double reg = 1e-12 * std::max(trace / A.rows(), 0.0);
  if (!(reg > 0.0)) {
    return false;
  }
  Eigen::SparseMatrix<double> identity(A.rows(), A.cols());
  identity.setIdentity();
  ldlt.compute(A + reg * identity);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }
  *x = ldlt.solve(b);
  return ldlt.info() == Eigen::Success && x->allFinite();
}

// Weighted least squares of the global system with the per-element weights w
// (triplets first, then links). Normal equations H^T W H x = H^T W d.
bool SolveGlobalLinear(const std::vector<ARTTriplet>& triplets,
                       const GlobalProblem& problem,
                       const GlobalLayout& layout,
                       const std::vector<double>& w,
                       Eigen::VectorXd* x) {
  const int nt = problem.NumTriplets();
  const int nl = static_cast<int>(problem.links.size());
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(nt * 6 * 3 + nl * 27);
  Eigen::VectorXd d = Eigen::VectorXd::Zero(6 * nt + 3 * nl);
  for (int i = 0; i < nt; ++i) {
    const ARTTriplet& t = triplets[problem.triplets[i]];
    const double sw = std::sqrt(w[i]);
    const int alpha_col = layout.alpha_col[i];
    for (int block = 0; block < 2; ++block) {
      const ViewId other = block == 0 ? t.zeta : t.nu;
      const Vector3d& b = block == 0 ? t.b_ez : t.b_en;
      const int col_eta = BlockOf(layout, t.eta);
      const int col_other = BlockOf(layout, other);
      for (int k = 0; k < 3; ++k) {
        const int row = 6 * i + 3 * block + k;
        if (col_eta >= 0) {
          entries.emplace_back(row, col_eta + k, sw);
        }
        if (col_other >= 0) {
          entries.emplace_back(row, col_other + k, -sw);
        }
        if (alpha_col >= 0) {
          entries.emplace_back(row, alpha_col, -sw * b(k));
        } else {
          d(row) = sw * b(k);  // seed: alpha = 1 moves to the right-hand side
        }
      }
    }
  }
  for (int n = 0; n < nl; ++n) {
    const TrackLink& link = problem.links[n];
    const double sw = std::sqrt(w[nt + n]);
    const Matrix3d blocks[3] = {link.B + link.C, -link.B, -link.C};
    const ViewId views[3] = {link.i, link.l, link.j};
    for (int b = 0; b < 3; ++b) {
      const int col = BlockOf(layout, views[b]);
      if (col < 0) {
        continue;
      }
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          if (blocks[b](r, c) != 0.0) {
            entries.emplace_back(6 * nt + 3 * n + r, col + c,
                                 sw * blocks[b](r, c));
          }
        }
      }
    }
  }
  Eigen::SparseMatrix<double> H(6 * nt + 3 * nl, layout.num_vars);
  H.setFromTriplets(entries.begin(), entries.end());
  const Eigen::SparseMatrix<double> Ht = H.transpose();
  const Eigen::SparseMatrix<double> A = Ht * H;
  const Eigen::VectorXd rhs = Ht * d;
  return SolveSymmetricPositiveDefinite(A, rhs, x);
}

// Huber weights with the scale multiplier * 1.4826 * median(rho) (or the
// fixed scale). Returns the scale, 0 if it is not positive.
double HuberWeights(const std::vector<double>& rho,
                    const ARTPositionEstimator::Options& opt,
                    std::vector<double>* w) {
  w->assign(rho.size(), 1.0);
  if (rho.empty()) {
    return 0.0;
  }
  const double sigma =
      opt.irls_scale > 0.0 ? opt.irls_scale
                           : opt.irls_scale_multiplier * 1.4826 * Median(rho);
  if (!(sigma > 0.0)) {
    return 0.0;
  }
  for (size_t e = 0; e < rho.size(); ++e) {
    (*w)[e] = rho[e] <= sigma ? 1.0 : sigma / rho[e];
  }
  return sigma;
}

// Solves the global system with IRLS (Huber) and the negative-scale rejection.
// The main triplets must contain `seed`. Fills positions (reference at the
// origin).
bool SolveGlobal(const std::vector<ARTTriplet>& triplets,
                 GlobalProblem problem,
                 const int seed,
                 const ViewId reference,
                 const ARTPositionEstimator::Options& opt,
                 std::unordered_map<ViewId, Vector3d>* positions) {
  GlobalLayout layout = MakeLayout(triplets, problem, seed, reference);
  std::vector<double> w(problem.NumElements(), 1.0);
  Eigen::VectorXd x;
  if (!SolveGlobalLinear(triplets, problem, layout, w, &x)) {
    return false;
  }

  for (int iter = 0; iter < opt.num_irls_iterations; ++iter) {
    const int nt = problem.NumTriplets();
    const int nl = static_cast<int>(problem.links.size());
    // Triplets: residual relative to the triplet scale. Links: residual of the
    // normalized constraint. Each kind gets its own Huber scale. The weights
    // are NOT divided by alpha^2: that rewards shrinking the triplet scales and
    // makes the IRLS collapse towards the trivial solution (observed on ETH3D
    // office).
    std::vector<double> rho_t(nt), rho_l(nl);
    for (int i = 0; i < nt; ++i) {
      rho_t[i] = TripletResidualNorm(triplets, problem, layout, x, i) /
                 std::max(std::abs(AlphaOf(layout, x, i)), kEps);
    }
    for (int n = 0; n < nl; ++n) {
      rho_l[n] = LinkResidualNorm(problem.links[n], layout, x);
    }
    std::vector<double> w_t, w_l;
    const double sigma = HuberWeights(rho_t, opt, &w_t);
    if (!(sigma > 0.0)) {
      break;
    }
    HuberWeights(rho_l, opt, &w_l);
    std::vector<double> new_w = w_t;
    new_w.insert(new_w.end(), w_l.begin(), w_l.end());
    Eigen::VectorXd x_new;
    if (!SolveGlobalLinear(triplets, problem, layout, new_w, &x_new)) {
      break;  // keep the previous solution
    }
    const double change = (x_new - x).norm() / std::max(x_new.norm(), kEps);
    if (!std::isfinite(change) || change > 1.0) {
      VLOG(2) << "ART IRLS iteration " << iter << " diverged (relative change "
              << change << "); keeping the previous solution.";
      break;
    }
    x = x_new;
    w = new_w;
    VLOG(2) << "ART IRLS iteration " << iter << ": scale " << sigma
            << ", relative change " << change;
    if (change < opt.irls_convergence_threshold) {
      break;
    }
  }

  // Negative scales (chirality inconsistency). Main triplets with alpha <= 0
  // are dropped and the part rigidly connected to the seed is kept. An
  // attached body is dropped as a whole if one of its triplets has a negative
  // scale or if a main view it is anchored to is gone. Then solve once more.
  const int nt = problem.NumTriplets();
  std::vector<int> main_kept;
  std::set<int> dropped_groups;
  int num_dropped = 0;
  for (int i = 0; i < nt; ++i) {
    const bool negative =
        problem.triplets[i] != seed && AlphaOf(layout, x, i) <= 0.0;
    num_dropped += negative ? 1 : 0;
    const int group = problem.triplet_group[i];
    if (group >= 0) {
      if (negative) {
        dropped_groups.insert(group);
      }
    } else if (!negative) {
      main_kept.push_back(problem.triplets[i]);
    }
  }
  VLOG(2) << "ART: negative-scale triplets dropped: " << num_dropped;
  if (num_dropped > 0) {
    std::set<ViewId> old_main_views, main_views;
    for (int i = 0; i < nt; ++i) {
      if (problem.triplet_group[i] < 0) {
        for (const ViewId v : TripletViews(triplets[problem.triplets[i]])) {
          old_main_views.insert(v);
        }
      }
    }
    const std::vector<int> component =
        EdgeConnectedComponent(triplets, main_kept, seed);
    for (const int t : component) {
      for (const ViewId v : TripletViews(triplets[t])) {
        main_views.insert(v);
      }
    }
    // Anchors of a body: the views of its elements that were main views.
    auto check_anchor = [&](const int group, const ViewId v) {
      if (group >= 0 && old_main_views.count(v) > 0 &&
          main_views.count(v) == 0) {
        dropped_groups.insert(group);
      }
    };
    for (int i = 0; i < nt; ++i) {
      for (const ViewId v : TripletViews(triplets[problem.triplets[i]])) {
        check_anchor(problem.triplet_group[i], v);
      }
    }
    for (size_t n = 0; n < problem.links.size(); ++n) {
      const TrackLink& link = problem.links[n];
      for (const ViewId v : {link.i, link.j, link.l}) {
        check_anchor(problem.link_group[n], v);
      }
    }
    std::unordered_map<int, double> main_weight;
    for (int i = 0; i < nt; ++i) {
      if (problem.triplet_group[i] < 0) {
        main_weight[problem.triplets[i]] = w[i];
      }
    }
    GlobalProblem reduced;
    std::vector<double> reduced_w;
    for (const int t : component) {
      reduced.triplets.push_back(t);
      reduced.triplet_group.push_back(-1);
      reduced_w.push_back(main_weight.at(t));
    }
    for (int i = 0; i < nt; ++i) {
      const int group = problem.triplet_group[i];
      if (group >= 0 && dropped_groups.count(group) == 0) {
        reduced.triplets.push_back(problem.triplets[i]);
        reduced.triplet_group.push_back(group);
        reduced_w.push_back(w[i]);
      }
    }
    for (size_t n = 0; n < problem.links.size(); ++n) {
      if (dropped_groups.count(problem.link_group[n]) == 0) {
        reduced.links.push_back(problem.links[n]);
        reduced.link_group.push_back(problem.link_group[n]);
        reduced_w.push_back(w[nt + n]);
      }
    }
    VLOG(2) << "ART: attached bodies dropped: " << dropped_groups.size();
    problem = std::move(reduced);
    w = std::move(reduced_w);
    layout = MakeLayout(triplets, problem, seed, reference);
    if (!SolveGlobalLinear(triplets, problem, layout, w, &x)) {
      return false;
    }
  }

  positions->clear();
  (*positions)[reference] = Vector3d::Zero();
  for (const auto& entry : layout.block) {
    (*positions)[entry.first] = Vector3d(x.segment<3>(entry.second));
  }
  return true;
}

// Result of the per-candidate evaluation.
struct CandidateResult {
  bool valid = false;
  ARTTriplet triplet;
};

CandidateResult EvaluateCandidate(
    const std::array<ViewId, 3>& views,
    const std::vector<TrackId>& tracks,
    const Reconstruction& reconstruction,
    const std::unordered_map<ViewId, Matrix3d>& rotations,
    const ARTPositionEstimator::Options& opt) {
  CandidateResult result;
  std::array<std::vector<Vector3d>, 3> rays;
  std::array<Matrix3d, 3> R;
  for (int v = 0; v < 3; ++v) {
    R[v] = rotations.at(views[v]);
    const View* view = reconstruction.View(views[v]);
    rays[v].reserve(tracks.size());
    for (const TrackId track : tracks) {
      rays[v].push_back(GetNormalizedFeatureRay(*view, track));
    }
  }

  // The three reference edges (a,b), (a,c), (b,c) with third view c, b, a.
  // Swapping eta and zeta gives the same solution, so these cover all cases.
  static const int kChoices[3][3] = {{0, 1, 2}, {0, 2, 1}, {1, 2, 0}};
  double best_ratio = std::numeric_limits<double>::infinity();
  ReferenceChoice best;
  bool found = false;
  for (const auto& c : kChoices) {
    ReferenceChoice choice;
    if (!SolveReferenceChoice(rays, R, c[0], c[1], c[2], opt, &choice)) {
      continue;
    }
    if (choice.solution.eigen_ratio < best_ratio) {
      best_ratio = choice.solution.eigen_ratio;
      best = choice;
      result.triplet.eta = views[c[0]];
      result.triplet.zeta = views[c[1]];
      result.triplet.nu = views[c[2]];
      found = true;
    }
  }
  if (!found || best_ratio > opt.max_eigen_ratio) {
    return result;
  }
  result.valid = true;
  result.triplet.b_ez = best.solution.b_ez;
  result.triplet.b_en = best.solution.b_en;
  result.triplet.conf = best.conf;
  result.triplet.eigen_ratio = best_ratio;
  return result;
}

std::vector<TrackId> IntersectSorted(const std::vector<TrackId>& a,
                                     const std::vector<TrackId>& b) {
  std::vector<TrackId> out;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                        std::back_inserter(out));
  return out;
}

}  // namespace

// ----------------------------------------------------------------------------
// Public triplet solver.

bool SolveARTTriplet(const std::vector<Eigen::Vector3d>& x_eta,
                     const std::vector<Eigen::Vector3d>& x_zeta,
                     const std::vector<Eigen::Vector3d>& x_nu,
                     const Eigen::Matrix3d& R_eta,
                     const Eigen::Matrix3d& R_zeta,
                     const Eigen::Matrix3d& R_nu,
                     const std::vector<double>& weights,
                     ARTTripletSolution* sol) {
  CHECK_NOTNULL(sol);
  const size_t n = x_eta.size();
  if (n == 0 || x_zeta.size() != n || x_nu.size() != n ||
      weights.size() != n) {
    return false;
  }
  std::vector<Matrix6x3d> B_k(n), C_k(n);
  for (size_t k = 0; k < n; ++k) {
    ComputeARTBlocks(x_eta[k], x_zeta[k], x_nu[k], R_eta, R_zeta, R_nu,
                     &B_k[k], &C_k[k]);
  }
  return SolveARTWithBlocks(x_eta, x_zeta, x_nu, R_eta, R_zeta, R_nu, B_k, C_k,
                            weights, kDefaultMinConditioning, sol);
}

bool SolveARTTripletFiltered(const std::vector<Eigen::Vector3d>& x_eta,
                             const std::vector<Eigen::Vector3d>& x_zeta,
                             const std::vector<Eigen::Vector3d>& x_nu,
                             const Eigen::Matrix3d& R_eta,
                             const Eigen::Matrix3d& R_zeta,
                             const Eigen::Matrix3d& R_nu,
                             const ARTPositionEstimator::Options& options,
                             ARTTripletSolution* sol,
                             std::vector<int>* kept) {
  CHECK_NOTNULL(sol);
  const size_t n = x_eta.size();
  if (n == 0 || x_zeta.size() != n || x_nu.size() != n) {
    return false;
  }
  const std::array<std::vector<Vector3d>, 3> rays = {x_eta, x_zeta, x_nu};
  const std::array<Matrix3d, 3> R = {R_eta, R_zeta, R_nu};
  ReferenceChoice choice;
  if (!SolveReferenceChoice(rays, R, 0, 1, 2, options, &choice)) {
    return false;
  }
  *sol = choice.solution;
  if (kept != nullptr) {
    *kept = choice.kept;
  }
  return true;
}

// ----------------------------------------------------------------------------
// ARTPositionEstimator.

ARTPositionEstimator::ARTPositionEstimator(const Options& options,
                                           const Reconstruction& reconstruction)
    : options_(options), reconstruction_(reconstruction) {
  CHECK_GT(options.num_threads, 0);
}

bool ARTPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations,
    std::unordered_map<ViewId, Vector3d>* positions) {
  CHECK_NOTNULL(positions)->clear();
  if (!rig_constraints().empty()) {
    VLOG(2) << "ART ignores the rig constraints.";
  }

  std::unordered_map<ViewId, Matrix3d> rotations;
  for (const auto& ori : orientations) {
    if (reconstruction_.View(ori.first) == nullptr) {
      continue;
    }
    rotations[ori.first] = AngleAxisToRotationMatrix(ori.second);
  }

  // Edges of the view graph between views with rotations.
  std::set<ViewIdPair> edges;
  for (const auto& view_pair : view_pairs) {
    const ViewId a = view_pair.first.first;
    const ViewId b = view_pair.first.second;
    if (a == b || !ContainsKey(rotations, a) || !ContainsKey(rotations, b)) {
      continue;
    }
    edges.insert(MakeEdge(a, b));
  }
  // Optionally add track co-visibility edges (view pairs sharing at least
  // min_tracks_per_triplet tracks). Tracks chain matches across views, so
  // triplets exist that are not triangles of the verified view graph; LiGT
  // uses them implicitly.
  if (options_.use_track_covisibility_triplets) {
    std::map<ViewIdPair, int> covisibility;
    for (const TrackId track_id : reconstruction_.TrackIds()) {
      const Track* track = reconstruction_.Track(track_id);
      std::vector<ViewId> views;
      for (const ViewId v : track->ViewIds()) {
        if (ContainsKey(rotations, v)) {
          views.push_back(v);
        }
      }
      if (views.size() < 3) {
        continue;
      }
      for (size_t i = 0; i < views.size(); ++i) {
        for (size_t j = i + 1; j < views.size(); ++j) {
          ++covisibility[MakeEdge(views[i], views[j])];
        }
      }
    }
    for (const auto& entry : covisibility) {
      if (entry.second >= options_.min_tracks_per_triplet) {
        edges.insert(entry.first);
      }
    }
  }
  std::map<ViewId, std::vector<ViewId>> neighbors;
  for (const ViewIdPair& e : edges) {
    neighbors[e.first].push_back(e.second);
    neighbors[e.second].push_back(e.first);
  }
  for (auto& entry : neighbors) {
    std::sort(entry.second.begin(), entry.second.end());
  }

  // Sorted track lists of the views.
  std::unordered_map<ViewId, std::vector<TrackId>> view_tracks;
  for (const auto& rotation : rotations) {
    std::vector<TrackId> ids =
        reconstruction_.View(rotation.first)->TrackIds();
    std::sort(ids.begin(), ids.end());
    view_tracks[rotation.first] = std::move(ids);
  }

  // Candidate triplets from the view graph (all three edges present), with
  // their shared tracks.
  using ViewTriple = std::array<ViewId, 3>;
  std::map<ViewTriple, std::vector<TrackId>> candidates;
  for (const ViewIdPair& e : edges) {
    const ViewId a = e.first;
    const ViewId b = e.second;
    for (const ViewId c : neighbors[b]) {
      if (c <= b || !edges.count(MakeEdge(a, c))) {
        continue;
      }
      std::vector<TrackId> shared =
          IntersectSorted(IntersectSorted(view_tracks[a], view_tracks[b]),
                          view_tracks[c]);
      if (static_cast<int>(shared.size()) >= options_.min_tracks_per_triplet) {
        candidates[ViewTriple{a, b, c}] = std::move(shared);
      }
    }
  }

  // Coverage fallback: views outside the largest edge-connected component of
  // the candidate triplets (e.g. in no candidate triplet at all) get candidate
  // triplets with any two views they share tracks with, at the lower
  // threshold min_tracks_per_triplet_fallback. Without this such views would
  // get no position at all.
  {
    // "Covered" = in the largest edge-connected component of the candidate
    // triplets. Triplets that touch the rest through one view only leave
    // their scale free and are dropped later, so their views count as
    // uncovered. Union-find over edges; a triplet unions its three edges.
    std::map<ViewIdPair, ViewIdPair> parent;
    std::function<ViewIdPair(const ViewIdPair&)> find =
        [&](const ViewIdPair& e) -> ViewIdPair {
      auto it = parent.find(e);
      if (it == parent.end()) {
        parent[e] = e;
        return e;
      }
      if (it->second == e) {
        return e;
      }
      const ViewIdPair root = find(it->second);
      parent[e] = root;
      return root;
    };
    for (const auto& candidate : candidates) {
      const ViewTriple& t = candidate.first;
      const ViewIdPair e0 = MakeEdge(t[0], t[1]);
      const ViewIdPair r0 = find(e0);
      parent[find(MakeEdge(t[0], t[2]))] = r0;
      parent[find(MakeEdge(t[1], t[2]))] = r0;
    }
    std::map<ViewIdPair, std::set<ViewId>> component_views;
    for (const auto& candidate : candidates) {
      const ViewTriple& t = candidate.first;
      component_views[find(MakeEdge(t[0], t[1]))].insert(t.begin(), t.end());
    }
    std::unordered_set<ViewId> covered;
    size_t largest = 0;
    for (const auto& entry : component_views) {
      if (entry.second.size() > largest) {
        largest = entry.second.size();
        covered = std::unordered_set<ViewId>(entry.second.begin(),
                                             entry.second.end());
      }
    }
    const int min_fallback = std::min(options_.min_tracks_per_triplet,
                                      options_.min_tracks_per_triplet_fallback);
    std::vector<ViewId> view_ids;
    for (const auto& rotation : rotations) {
      view_ids.push_back(rotation.first);
    }
    std::sort(view_ids.begin(), view_ids.end());
    for (const ViewId a : view_ids) {
      if (covered.count(a) > 0 || min_fallback < 1) {
        continue;
      }
      // Views sharing enough tracks with a.
      std::vector<std::pair<ViewId, std::vector<TrackId>>> partners;
      for (const ViewId b : view_ids) {
        if (b == a) {
          continue;
        }
        std::vector<TrackId> shared_ab =
            IntersectSorted(view_tracks[a], view_tracks[b]);
        if (static_cast<int>(shared_ab.size()) >= min_fallback) {
          partners.emplace_back(b, std::move(shared_ab));
        }
      }
      int added = 0;
      for (size_t i = 0; i < partners.size(); ++i) {
        for (size_t j = i + 1; j < partners.size(); ++j) {
          std::vector<TrackId> shared =
              IntersectSorted(partners[i].second, view_tracks[partners[j].first]);
          if (static_cast<int>(shared.size()) < min_fallback) {
            continue;
          }
          std::array<ViewId, 3> key{a, partners[i].first, partners[j].first};
          std::sort(key.begin(), key.end());
          if (!ContainsKey(candidates, key)) {
            candidates[key] = std::move(shared);
            ++added;
          }
        }
      }
      VLOG(2) << "ART: view " << a << " outside the main triplet component; added "
              << added << " fallback triplets.";
    }
  }

  // Rank by the number of shared tracks and cap the triplets per view.
  std::vector<std::pair<ViewTriple, std::vector<TrackId>>> ranked(
      candidates.begin(), candidates.end());
  std::stable_sort(ranked.begin(), ranked.end(),
                   [](const std::pair<ViewTriple, std::vector<TrackId>>& x,
                      const std::pair<ViewTriple, std::vector<TrackId>>& y) {
                     return x.second.size() > y.second.size();
                   });
  std::unordered_map<ViewId, int> triplets_per_view;
  std::vector<std::pair<ViewTriple, std::vector<TrackId>>> kept;
  for (auto& candidate : ranked) {
    bool has_room = false;
    for (const ViewId v : candidate.first) {
      if (triplets_per_view[v] < options_.max_triplets_per_view) {
        has_room = true;
      }
    }
    if (!has_room) {
      continue;
    }
    for (const ViewId v : candidate.first) {
      ++triplets_per_view[v];
    }
    if (static_cast<int>(candidate.second.size()) >
        options_.max_tracks_per_triplet) {
      candidate.second.resize(options_.max_tracks_per_triplet);
    }
    kept.push_back(std::move(candidate));
  }
  VLOG(2) << "ART: evaluating " << kept.size() << " candidate triplets.";

  // Per-triplet solves. Results are stored by candidate index, so the output
  // does not depend on the thread order.
  std::vector<CandidateResult> results(kept.size());
  ParallelFor(options_.num_threads, kept.size(), [&](const size_t i) {
    results[i] = EvaluateCandidate(kept[i].first, kept[i].second,
                                   reconstruction_, rotations, options_);
  });

  std::vector<ARTTriplet> triplets;
  for (const CandidateResult& result : results) {
    if (result.valid) {
      triplets.push_back(result.triplet);
    }
  }
  {
    std::set<ViewId> vs;
    for (const auto& item : triplets) {
      for (const ViewId v : TripletViews(item)) vs.insert(v);
    }
    VLOG(2) << "ART: valid triplet solves: " << triplets.size() << " triplets, " << vs.size() << " views.";
  }
  if (triplets.empty()) {
    LOG(WARNING) << "ART found no valid view triplet.";
    return false;
  }

  // Triplet selection (Alg. 3 step 3); returns the indices of the triplets of
  // the largest edge-connected component, best confidence first.
  auto select_active = [this](const std::vector<ARTTriplet>& triplets) {
  const int num_triplets = static_cast<int>(triplets.size());
  std::vector<int> order(num_triplets);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&triplets](int a, int b) {
    return triplets[a].conf < triplets[b].conf;
  });
  std::vector<double> confs;
  confs.reserve(num_triplets);
  for (const ARTTriplet& t : triplets) {
    confs.push_back(t.conf);
  }
  const double tau = std::max(
      options_.triplet_conf_factor * Median(confs), kTinyConfidence);
  VLOG(3) << "ART: confidence threshold " << tau;

  EdgeComponents components;
  std::vector<bool> is_kept(num_triplets, false);
  std::unordered_set<ViewId> covered;
  auto add = [&](const int t) {
    is_kept[t] = true;
    components.AddTriplet(triplets[t]);
    for (const ViewId v : TripletViews(triplets[t])) {
      covered.insert(v);
    }
  };
  for (const int t : order) {
    if (triplets[t].conf <= tau) {
      add(t);
    }
  }
  // Greedily add back rejected triplets that link a new view or merge two
  // edge-connected components.
  for (const int t : order) {
    if (is_kept[t]) {
      continue;
    }
    bool new_view = false;
    for (const ViewId v : TripletViews(triplets[t])) {
      if (covered.count(v) == 0) {
        new_view = true;
      }
    }
    std::set<int> roots;
    for (const ViewIdPair& e : TripletEdges(triplets[t])) {
      const int id = components.FindEdge(e);
      if (id >= 0) {
        roots.insert(components.Root(id));
      }
    }
    if (new_view || roots.size() >= 2) {
      add(t);
    }
  }

  std::vector<int> kept_order;
  for (const int t : order) {
    if (is_kept[t]) {
      kept_order.push_back(t);
    }
  }

  // The largest rigid component of kept_order, by number of views (ties: the
  // component of the best-confidence triplet). Fills the component's views.
  auto largest_component = [&triplets](const std::vector<int>& kept_order,
                                       std::set<ViewId>* main_views,
                                       std::vector<int>* component_of,
                                       int* best_root) {
    *component_of = RigidComponents(triplets, kept_order);
    std::unordered_map<int, std::set<ViewId>> component_views;
    for (size_t i = 0; i < kept_order.size(); ++i) {
      for (const ViewId v : TripletViews(triplets[kept_order[i]])) {
        component_views[(*component_of)[i]].insert(v);
      }
    }
    size_t best_size = 0;
    for (const auto& entry : component_views) {
      best_size = std::max(best_size, entry.second.size());
    }
    *best_root = -1;
    for (size_t i = 0; i < kept_order.size(); ++i) {
      if (component_views[(*component_of)[i]].size() == best_size) {
        *best_root = (*component_of)[i];
        break;
      }
    }
    *main_views = component_views[*best_root];
    return component_views;
  };

  // Rigid growth. A valid triplet t that is not kept (e.g. rejected by its
  // confidence) is added if it attaches new views rigidly to the main
  // component: either t itself shares two views with it, or t joins a body
  // B (another rigid component, sharing an edge or two views with t) and
  // B + t shares at least two views with it. Best confidence first, one
  // triplet at a time, until nothing changes.
  std::set<ViewId> main_views;
  std::vector<int> component_of;
  int best_root = -1;
  while (true) {
    const auto component_views =
        largest_component(kept_order, &main_views, &component_of, &best_root);
    int to_add = -1;
    for (const int t : order) {
      if (is_kept[t]) {
        continue;
      }
      const auto views = TripletViews(triplets[t]);
      int num_main = 0;
      for (const ViewId v : views) {
        num_main += main_views.count(v) > 0 ? 1 : 0;
      }
      if (num_main == 0 || num_main == 3) {
        continue;
      }
      if (num_main == 2) {
        to_add = t;
        break;
      }
      // num_main == 1: look for a body that t joins rigidly.
      for (const auto& entry : component_views) {
        if (entry.first == best_root) {
          continue;
        }
        const std::set<ViewId>& body = entry.second;
        int num_body = 0;
        for (const ViewId v : views) {
          num_body += body.count(v) > 0 ? 1 : 0;
        }
        if (num_body < 2) {
          continue;  // t is not rigidly joined to this body
        }
        std::set<ViewId> joined = body;
        joined.insert(views.begin(), views.end());
        int shared = 0;
        bool new_view = false;
        for (const ViewId v : joined) {
          if (main_views.count(v) > 0) {
            ++shared;
          } else {
            new_view = true;
          }
        }
        if (shared >= 2 && new_view) {
          to_add = t;
          break;
        }
      }
      if (to_add >= 0) {
        break;
      }
    }
    if (to_add < 0) {
      break;
    }
    is_kept[to_add] = true;
    kept_order.push_back(to_add);
  }

  std::vector<int> active;
  for (size_t i = 0; i < kept_order.size(); ++i) {
    if (component_of[i] == best_root) {
      active.push_back(kept_order[i]);
    }
  }
  return active;
  };

  std::vector<int> active = select_active(triplets);
  auto active_views = [&]() {
    std::set<ViewId> vs;
    for (const int t : active) {
      for (const ViewId v : TripletViews(triplets[t])) vs.insert(v);
    }
    return vs;
  };
  VLOG(2) << "ART: largest rigid component: " << active.size()
          << " triplets, " << active_views().size() << " views.";
  if (VLOG_IS_ON(3)) {
    const std::set<ViewId> covered_views = active_views();
    std::ostringstream cv;
    for (const ViewId v : covered_views) cv << v << " ";
    VLOG(3) << "ART: covered views: " << cv.str();
    for (const ARTTriplet& t : triplets) {
      const auto views = TripletViews(t);
      if (covered_views.count(views[0]) && covered_views.count(views[1]) &&
          covered_views.count(views[2])) {
        continue;
      }
      VLOG(3) << "ART: uncovered triplet (" << views[0] << ", " << views[1]
              << ", " << views[2] << ") conf " << t.conf << " ratio "
              << t.eigen_ratio;
    }
  }

  // Bridging: views outside the component get triplets that share an edge
  // with it (see Options::num_coverage_rounds).
  {
    std::set<ViewTriple> evaluated;
    for (const auto& candidate : kept) {
      evaluated.insert(candidate.first);
    }
    const int min_bridge = std::max(
        1, std::min(options_.min_tracks_per_triplet,
                    options_.min_tracks_per_triplet_fallback));
    std::vector<ViewId> all_views;
    for (const auto& rotation : rotations) {
      all_views.push_back(rotation.first);
    }
    std::sort(all_views.begin(), all_views.end());
    for (int round = 0; round < options_.num_coverage_rounds &&
                        options_.min_tracks_per_triplet_fallback > 0;
         ++round) {
      const std::set<ViewId> covered_views = active_views();
      std::set<ViewIdPair> active_edges;
      for (const int t : active) {
        for (const ViewIdPair& e : TripletEdges(triplets[t])) {
          active_edges.insert(e);
        }
      }
      std::vector<std::pair<ViewTriple, std::vector<TrackId>>> bridges;
      for (const ViewId a : all_views) {
        if (covered_views.count(a) > 0) {
          continue;
        }
        // Tracks a shares with each covered view.
        std::unordered_map<ViewId, std::vector<TrackId>> shared_with;
        for (const ViewId u : covered_views) {
          std::vector<TrackId> s = IntersectSorted(view_tracks[a], view_tracks[u]);
          if (static_cast<int>(s.size()) >= min_bridge) {
            shared_with[u] = std::move(s);
          }
        }
        std::vector<std::pair<ViewTriple, std::vector<TrackId>>> mine;
        for (const ViewIdPair& e : active_edges) {
          const auto it = shared_with.find(e.first);
          if (it == shared_with.end() || !ContainsKey(shared_with, e.second)) {
            continue;
          }
          std::vector<TrackId> shared =
              IntersectSorted(it->second, view_tracks[e.second]);
          if (static_cast<int>(shared.size()) < min_bridge) {
            continue;
          }
          ViewTriple key{a, e.first, e.second};
          std::sort(key.begin(), key.end());
          if (evaluated.count(key) > 0) {
            continue;
          }
          if (static_cast<int>(shared.size()) > options_.max_tracks_per_triplet) {
            shared.resize(options_.max_tracks_per_triplet);
          }
          mine.emplace_back(key, std::move(shared));
        }
        std::stable_sort(mine.begin(), mine.end(),
                         [](const std::pair<ViewTriple, std::vector<TrackId>>& x,
                            const std::pair<ViewTriple, std::vector<TrackId>>& y) {
                           return x.second.size() > y.second.size();
                         });
        if (static_cast<int>(mine.size()) > options_.max_triplets_per_view) {
          mine.resize(options_.max_triplets_per_view);
        }
        for (auto& m : mine) {
          evaluated.insert(m.first);
          bridges.push_back(std::move(m));
        }
      }
      if (bridges.empty()) {
        break;
      }
      std::vector<CandidateResult> bridge_results(bridges.size());
      ParallelFor(options_.num_threads, bridges.size(),
                  [&](const size_t i) {
                    bridge_results[i] =
                        EvaluateCandidate(bridges[i].first, bridges[i].second,
                                          reconstruction_, rotations, options_);
                  });
      int num_valid = 0;
      for (const CandidateResult& result : bridge_results) {
        if (result.valid) {
          triplets.push_back(result.triplet);
          ++num_valid;
        }
      }
      VLOG(2) << "ART: coverage round " << round << ": " << bridges.size()
              << " bridge triplets, " << num_valid << " valid.";
      if (num_valid == 0) {
        break;
      }
      active = select_active(triplets);
      VLOG(2) << "ART: component after bridging: " << active.size()
              << " triplets, " << active_views().size() << " views.";
    }
  }
  // Attach the rest with LiGT track constraints. Views outside the main rigid
  // component (bodies: rigid components of the remaining valid triplets, or
  // single views) cannot be attached by triplets, e.g. a group of views that
  // shares a single view with the main component. A body is attached with the
  // LiGT constraints of the tracks it shares with the main component (each
  // constraint involves at least one body view), if there are at least
  // min_bridge_tracks such tracks. LiGT constraints are linear in the centers
  // and need no scale unknown, so a track seen in one body view and two main
  // views already constrains the body.
  GlobalProblem problem;
  for (const int t : active) {
    problem.triplets.push_back(t);
    problem.triplet_group.push_back(-1);
  }
  if (options_.min_bridge_tracks > 0) {
    std::set<ViewId> main_views = active_views();
    std::vector<bool> in_active(triplets.size(), false);
    for (const int t : active) {
      in_active[t] = true;
    }
    std::vector<int> rest;
    for (int t = 0; t < static_cast<int>(triplets.size()); ++t) {
      if (in_active[t]) {
        continue;
      }
      int num_main = 0;
      for (const ViewId v : TripletViews(triplets[t])) {
        num_main += main_views.count(v) > 0 ? 1 : 0;
      }
      if (num_main < 3) {
        rest.push_back(t);
      }
    }
    struct Body {
      std::vector<int> triplets;
      std::set<ViewId> views;
      bool attached = false;
    };
    std::vector<Body> bodies;
    {
      const std::vector<int> component_of = RigidComponents(triplets, rest);
      std::map<int, int> body_of_component;
      std::set<ViewId> in_body;
      for (size_t i = 0; i < rest.size(); ++i) {
        const auto inserted = body_of_component.emplace(
            component_of[i], static_cast<int>(bodies.size()));
        if (inserted.second) {
          bodies.emplace_back();
        }
        Body& body = bodies[inserted.first->second];
        body.triplets.push_back(rest[i]);
        for (const ViewId v : TripletViews(triplets[rest[i]])) {
          body.views.insert(v);
          in_body.insert(v);
        }
      }
      std::vector<ViewId> view_ids;
      for (const auto& rotation : rotations) {
        view_ids.push_back(rotation.first);
      }
      std::sort(view_ids.begin(), view_ids.end());
      for (const ViewId v : view_ids) {
        if (main_views.count(v) == 0 && in_body.count(v) == 0) {
          bodies.emplace_back();
          bodies.back().views.insert(v);
        }
      }
    }

    int num_attached_bodies = 0;
    int num_attached_views = 0;
    bool attached_any = true;
    while (attached_any) {
      attached_any = false;
      for (size_t b = 0; b < bodies.size(); ++b) {
        Body& body = bodies[b];
        if (body.attached) {
          continue;
        }
        std::set<ViewId> outside;
        for (const ViewId v : body.views) {
          if (main_views.count(v) == 0) {
            outside.insert(v);
          }
        }
        if (outside.empty()) {
          body.attached = true;
          continue;
        }
        // Tracks of the outside views, in track id order.
        std::vector<TrackId> track_ids;
        for (const ViewId u : outside) {
          track_ids.insert(track_ids.end(), view_tracks[u].begin(),
                           view_tracks[u].end());
        }
        std::sort(track_ids.begin(), track_ids.end());
        track_ids.erase(std::unique(track_ids.begin(), track_ids.end()),
                        track_ids.end());
        std::vector<TrackLink> links;
        int num_bridge_tracks = 0;
        for (const TrackId track_id : track_ids) {
          if (num_bridge_tracks >= options_.max_tracks_per_triplet) {
            break;
          }
          std::vector<ViewId> views;
          bool has_outside = false, has_main = false;
          for (const ViewId v : reconstruction_.Track(track_id)->ViewIds()) {
            const bool is_main = main_views.count(v) > 0;
            const bool is_outside = outside.count(v) > 0;
            if (is_main || body.views.count(v) > 0) {
              views.push_back(v);
              has_main |= is_main;
              has_outside |= is_outside;
            }
          }
          if (views.size() < 3 || !has_main || !has_outside) {
            continue;
          }
          std::sort(views.begin(), views.end());
          std::vector<Vector3d> rays(views.size());
          for (size_t k = 0; k < views.size(); ++k) {
            rays[k] = GetNormalizedFeatureRay(*reconstruction_.View(views[k]),
                                              track_id);
          }
          // Base views: the pair with the largest parallax (LiGT Eq. 29).
          size_t base1 = 0, base2 = 1;
          double best_theta = -1.0;
          for (size_t a = 0; a < views.size(); ++a) {
            for (size_t c = a + 1; c < views.size(); ++c) {
              const double theta = GetThetaSq(
                  rays[a], rays[c],
                  GetRij(rotations.at(views[a]), rotations.at(views[c])));
              if (theta > best_theta) {
                best_theta = theta;
                base1 = a;
                base2 = c;
              }
            }
          }
          bool used = false;
          for (size_t k = 0; k < views.size(); ++k) {
            if (k == base1 || k == base2) {
              continue;
            }
            if (outside.count(views[k]) == 0 &&
                outside.count(views[base1]) == 0 &&
                outside.count(views[base2]) == 0) {
              continue;  // a main-only constraint
            }
            // Paper notation as in LiGT: j = base1, l = current, i = base2.
            TrackLink link;
            link.i = views[base2];
            link.j = views[base1];
            link.l = views[k];
            ComputeLiGTBC(rays[base2], rays[base1], rays[k],
                          rotations.at(link.i), rotations.at(link.j),
                          rotations.at(link.l), &link.B, &link.C);
            const double norm =
                std::sqrt((link.B + link.C).squaredNorm() +
                          link.B.squaredNorm() + link.C.squaredNorm());
            if (!(norm > kMinBlockScale) || !std::isfinite(norm)) {
              continue;
            }
            link.B /= norm;
            link.C /= norm;
            links.push_back(link);
            used = true;
          }
          num_bridge_tracks += used ? 1 : 0;
        }
        if (num_bridge_tracks < options_.min_bridge_tracks) {
          continue;
        }
        const int group = static_cast<int>(b);
        for (const int t : body.triplets) {
          problem.triplets.push_back(t);
          problem.triplet_group.push_back(group);
        }
        for (const TrackLink& link : links) {
          problem.links.push_back(link);
          problem.link_group.push_back(group);
        }
        main_views.insert(body.views.begin(), body.views.end());
        body.attached = true;
        attached_any = true;
        ++num_attached_bodies;
        num_attached_views += static_cast<int>(outside.size());
      }
    }
    VLOG(2) << "ART: attached " << num_attached_bodies << " bodies ("
            << num_attached_views << " views) with " << problem.links.size()
            << " track constraints.";
  }

  // Reference view: the view in the most active triplets (ties: lowest id).
  std::map<ViewId, int> view_count;
  for (const int t : active) {
    for (const ViewId v : TripletViews(triplets[t])) {
      ++view_count[v];
    }
  }
  ViewId reference = kInvalidViewId;
  int reference_count = 0;
  for (const auto& entry : view_count) {
    if (entry.second > reference_count) {
      reference = entry.first;
      reference_count = entry.second;
    }
  }
  // Seed triplet (alpha = 1): the best-confidence triplet containing the
  // reference view.
  int seed = -1;
  for (const int t : active) {
    const auto views = TripletViews(triplets[t]);
    if (std::find(views.begin(), views.end(), reference) != views.end()) {
      seed = t;
      break;
    }
  }
  if (seed < 0 || reference == kInvalidViewId) {
    LOG(WARNING) << "ART: no seed triplet found.";
    return false;
  }
  VLOG(2) << "ART: " << active.size() << " triplets used, reference view "
          << reference << ".";

  if (!SolveGlobal(triplets, problem, seed, reference, options_, positions)) {
    LOG(WARNING) << "ART: the global linear system could not be solved.";
    positions->clear();
    return false;
  }

  // The sign of the solution is ambiguous: correct it with the relative
  // translations of the view pairs (as in LiGT).
  if (PositionsHaveWrongSign(view_pairs, rotations, *positions)) {
    for (auto& position : *positions) {
      position.second *= -1.0;
    }
  }
  return true;
}

std::unordered_map<ViewId, Vector3d> ARTPositionEstimator::EstimatePositionsWrapper(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientation) {
  std::unordered_map<ViewId, Vector3d> positions;
  EstimatePositions(view_pairs, orientation, &positions);
  return positions;
}

}  // namespace theia
