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

// An OSE objective. Implementations provide the per-observation model and its
// derivative with respect to the camera; camera-only terms (such as relative
// rotations) are evaluated through EvaluateRotationTerm. Adding a formulation
// means implementing this class and adding a case to CreateOseFormulation.
class OseFormulation {
 public:
  virtual ~OseFormulation() = default;

  virtual OseFormulationType Type() const = 0;

  virtual int NumResidualsPerObservation() const = 0;

  // Fills G and z for camera P and normalized measurement m.
  virtual void ObservationModel(const OseCamera& P,
                                const Eigen::Vector2d& m,
                                OseObservationModel* model) const = 0;

  // d r / d vec(P) (row-major vec) at point X.
  virtual void CameraJacobian(const OseCamera& P,
                              const Eigen::Vector2d& m,
                              const Eigen::Vector3d& X,
                              OseCameraJacobian* jacobian) const = 0;

  // Whether the problem's rotation terms are part of the objective.
  virtual bool UsesRotationTerms() const { return false; }

  // Residual (9) and Jacobians (9x12) of a rotation term. For a == b only
  // jacobian_a is used and contains the full derivative.
  static void EvaluateRotationTerm(
      const OseRotationTerm& term,
      const OseCamera& Pa,
      const OseCamera& Pb,
      Eigen::Matrix<double, 9, 1>* residual,
      Eigen::Matrix<double, 9, kOseCameraParameters, Eigen::RowMajor>*
          jacobian_a,
      Eigen::Matrix<double, 9, kOseCameraParameters, Eigen::RowMajor>*
          jacobian_b);
};

// Returns nullptr and sets `error` for formulations that are not implemented.
std::unique_ptr<OseFormulation> CreateOseFormulation(
    const InitFreeOptions& options, std::string* error);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_OSE_FORMULATION_H_
