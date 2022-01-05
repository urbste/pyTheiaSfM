#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace theia {

using Matrix99 = Eigen::Matrix<double, 9, 9>;
using Matrix91 = Eigen::Matrix<double, 9, 1>;
using Matrix31 = Eigen::Matrix<double, 3, 1>;
using Matrix33 = Eigen::Matrix<double, 3, 3>;
using Matrix39 = Eigen::Matrix<double, 3, 9>;
using Matrix93 = Eigen::Matrix<double, 9, 3>;
using Matrix96 = Eigen::Matrix<double, 9, 6>;
using Matrix66 = Eigen::Matrix<double, 6, 6>;

enum class NearestRotationMethod { FOAM, SVD };

const double DEFAULT_RANK_TOLERANCE = 1e-7;
const double DEFAULT_SQP_SQUARED_TOLERANCE = 1e-10;
const double DEFAULT_SQP_DET_THRESHOLD = 1.001;
const NearestRotationMethod DEFAULT_NEAREST_ROTATION_METHOD =
    NearestRotationMethod::SVD;
const double DEFAULT_ORTHOGONALITY_SQUARED_ERROR_THRESHOLD = 1e-8;
const double DEFAULT_EQUAL_VECTORS_SQUARED_DIFF = 1e-10;
const double DEFAULT_EQUAL_SQUARED_ERRORS_DIFF = 1e-6;
const double DEFAULT_POINT_VARIANCE_THRESHOLD = 1e-5;
const double SQRT3 = std::sqrt(3);

struct SQPSolution {
  Matrix91 r;      // Actual matrix upon convergence
  Matrix91 r_hat;  // "Clean" (nearest) rotation matrix
  Matrix31 t;
  int num_iterations;
  double sq_error;
};

void HandleSolution(const Matrix99& Omega,
                    const Eigen::Vector3d& point_mean,
                    SQPSolution& solution,
                    SQPSolution *solutions,
                    double& min_sq_error,
                    int& num_solutions);

double AverageSquaredProjectionError(
    const SQPSolution& solution,
    const std::vector<Eigen::Vector2d>& projections,
    const std::vector<Eigen::Vector3d>& points);

std::vector<double> AverageSquaredProjectionErrors(
    const std::vector<SQPSolution>& solutions,
    const std::vector<Eigen::Vector2d>& projections,
    const std::vector<Eigen::Vector3d>& points);

// Test cheirality for a given solution
bool TestPositiveDepth(const SQPSolution& solution,
                       const Eigen::Vector3d& point_mean);

// Determinant of 3x3 matrix stored as a 9x1 vector in a vector in ROW-MAJOR
// order
inline double Determinant9x1(const Matrix91& r) {
  return r[0] * r[4] * r[8] + r[1] * r[5] * r[6] + r[2] * r[3] * r[7] -
         r[6] * r[4] * r[2] - r[7] * r[5] * r[0] - r[8] * r[3] * r[1];
}

// Determinant of 3x3 matrix
inline double Determinant3x3(const Matrix33& M) {
  return M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) -
         M(0, 1) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0)) +
         M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0));
}

void SolveSQPSystem(const Matrix91& r, const Matrix99& Omega, Matrix91& delta);

//
// Invert a 3x3 symmetrix matrix (using low triangle values only)
bool InvertSymmetric3x3(const Matrix33& Q,
                        Matrix33& Qinv,
                        const double det_threshold = 1e-8);

// Simple SVD - based nearest rotation matrix. Argument should be a ROW-MAJOR
// matrix representation. Returns a ROW-MAJOR vector representation of the
// nearest rotation matrix.
void NearestRotationMatrix_SVD(const Matrix91& e, Matrix91& r);

// faster nearest rotation computation based on FOAM (see:
// http://users.ics.forth.gr/~lourakis/publ/2018_iros.pdf )
/* Solve the nearest orthogonal approximation problem
 * i.e., given B, find R minimizing ||R-B||_F
 *
 * The computation borrows from Markley's FOAM algorithm
 * "Attitude Determination Using Vector Observations: A Fast Optimal Matrix
 * Algorithm", J. Astronaut. Sci.
 *
 * See also M. Lourakis: "An Efficient Solution to Absolute Orientation", ICPR
 * 2016
 *
 *  Copyright (C) 2019 Manolis Lourakis (lourakis **at** ics forth gr)
 *  Institute of Computer Science, Foundation for Research & Technology - Hellas
 *  Heraklion, Crete, Greece.
 */
void NearestRotationMatrix_FOAM(const Matrix91& e, Matrix91& r);

double OrthogonalityError(const Matrix91& a);

//
// Compute the 3D null space (N) and 6D normal space (H) of the constraint
// Jacobian at a 9D vector r (not necessarilly a rotation-yet it should be
// rank-3)
void RowAndNullSpace(const Matrix91& r,
                     Matrix96& H,
                     Matrix93& N,
                     Matrix66& K,
                     const double norm_threhsold = 0.1);

}  // namespace theia
