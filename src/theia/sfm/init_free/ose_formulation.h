// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_OSE_FORMULATION_H_
#define THEIA_SFM_INIT_FREE_OSE_FORMULATION_H_

#include <Eigen/Core>
#include <memory>
#include <string>

#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/ose_problem.h"

namespace theia {

// Maximum number of residuals one observation contributes. pOSE uses 4.
constexpr int kMaxOseResidualsPerObservation = 4;
constexpr int kOseCameraParameters = 12;

// The residual of one observation is linear in the 3D point X (with the
// homogeneous point [X; 1]):
//
//   r(P, X) = G(P, m) X - z(P, m).
//
// This separable structure is what VarPro exploits: for fixed cameras each
// point is a small linear least-squares problem.
struct OseObservationModel {
  Eigen::Matrix<double, Eigen::Dynamic, 3, 0, kMaxOseResidualsPerObservation, 3>
      G;
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxOseResidualsPerObservation, 1>
      z;
};

using OseCameraJacobian = Eigen::Matrix<double,
                                        Eigen::Dynamic,
                                        kOseCameraParameters,
                                        Eigen::RowMajor,
                                        kMaxOseResidualsPerObservation,
                                        kOseCameraParameters>;

using OseRotationJacobian =
    Eigen::Matrix<double, 9, kOseCameraParameters, Eigen::RowMajor>;

// An OSE objective. Implementations provide the per-observation model and its
// derivative with respect to the camera; camera-only terms (relative
// rotations) are evaluated through EvaluateRotationTerm. Adding a formulation
// means implementing this class and adding a case to CreateOseFormulation.
//
// Some formulations (expOSE) minimize a least-squares model that is expanded
// around a per-observation linearization value `state`. The solver keeps one
// double per observation, initializes it with InitialState and, from
// iteration UpdateStateFromIteration() on, replaces it with UpdatedState
// after every accepted step. Stateless formulations ignore it.
class OseFormulation {
 public:
  virtual ~OseFormulation() = default;

  virtual OseFormulationType Type() const = 0;

  virtual int NumResidualsPerObservation() const = 0;

  // Fills G and z for camera P and normalized measurement m.
  virtual void ObservationModel(const OseCamera& P,
                                const Eigen::Vector2d& m,
                                double state,
                                OseObservationModel* model) const = 0;

  // d r / d vec(P) (row-major vec) at point X, with `state` held fixed.
  virtual void CameraJacobian(const OseCamera& P,
                              const Eigen::Vector2d& m,
                              const Eigen::Vector3d& X,
                              double state,
                              OseCameraJacobian* jacobian) const = 0;

  // Per-observation linearization state (see the class comment).
  virtual bool HasState() const { return false; }
  virtual double InitialState(const Eigen::Vector2d& m) const { return 0.0; }
  virtual double UpdatedState(const OseCamera& P,
                              const Eigen::Vector2d& m,
                              const Eigen::Vector3d& X) const {
    return 0.0;
  }
  virtual int UpdateStateFromIteration() const { return 0; }

  // Applied to every initial camera before the solve (expOSE flips cameras
  // with det(A) < 0, as in InitFreeBA).
  virtual void NormalizeInitialCamera(OseCamera* P) const {}

  // Radial formulations estimate only the first two rows of P.
  virtual bool IsRadial() const { return false; }

  // Whether the problem's rotation terms are part of the objective.
  virtual bool UsesRotationTerms() const { return false; }

  // Residual (9) and Jacobians (9x12) of a rotation term, including
  // sqrt(weight) * sqrt_weight_matrix. For a == b only jacobian_a is used and
  // contains the full derivative.
  static void EvaluateRotationTerm(const OseRotationTerm& term,
                                   const OseCamera& Pa,
                                   const OseCamera& Pb,
                                   Eigen::Matrix<double, 9, 1>* residual,
                                   OseRotationJacobian* jacobian_a,
                                   OseRotationJacobian* jacobian_b);
};

// Returns nullptr and sets `error` for formulations that are not implemented.
std::unique_ptr<OseFormulation> CreateOseFormulation(
    const InitFreeOptions& options, std::string* error);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_OSE_FORMULATION_H_
