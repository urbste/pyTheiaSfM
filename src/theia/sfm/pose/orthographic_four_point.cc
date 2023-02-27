// Copyright (C) 2022
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com

#include "theia/sfm/pose/orthographic_four_point.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>

#include "theia/sfm/pose/util.h"
#include "theia/math/util.h"

namespace theia {

bool PlanarUncalibratedOrthographicPose(
        const std::vector<FeatureCorrespondence2D3D>& correspondences,
        const Eigen::Vector2d &principal_point,
        std::vector<Eigen::Matrix3d>* solution_rotations,
        std::vector<Eigen::Vector3d>* solution_translations,
        double* magnification) {

  std::vector<Eigen::Vector2d> feature_points(correspondences.size());
  std::vector<Eigen::Vector3d> world_points(correspondences.size());

  for (size_t i=0; i < correspondences.size(); ++i) {
    feature_points[i] = correspondences[i].feature;
    world_points[i] = correspondences[i].world_point;
  }

  return PlanarUncalibratedOrthographicPose(
    feature_points, world_points,
    principal_point, solution_rotations, solution_translations,
    magnification
  );
}

bool PlanarUncalibratedOrthographicPose(
        const std::vector<Eigen::Vector2d>& feature_point,
        const std::vector<Eigen::Vector3d>& world_point,
        const Eigen::Vector2d& principal_point,
        std::vector<Eigen::Matrix3d>* solution_rotations,
        std::vector<Eigen::Vector3d>* solution_translations,
        double* magnification) {
    Eigen::MatrixXd pts2d;
    Eigen::MatrixXd pts3d;

    const size_t num_pts = feature_point.size();

    if (num_pts < 3) {
        LOG(INFO) << "More than 3 points are needed.";
        return false;
    }

    pts2d.resize(2*num_pts,1);
    pts3d.resize(2*num_pts,6);
    pts2d.setZero();
    pts3d.setZero();

    for (size_t i = 0; i < num_pts; ++i) {
        pts2d(i,0) = feature_point[i][0];
        pts2d(i+num_pts,0) = feature_point[i][1];
        Eigen::Vector3d world_pt = world_point[i];
        world_pt[2] = 1.0; // make sure this is set to 1
        pts3d.block<1,3>(i,0) = world_pt;
        pts3d.block<1,3>(i+num_pts,3) = world_pt;
    }

    Eigen::MatrixXd h = PseudoInverse(pts3d) * pts2d;
    // ortho homography
    Eigen::Matrix3d H;

    H<<h(0,0), h(1,0), h(2,0),
        h(3,0), h(4,0), h(5,0),
        0.0, 0.0, 1.0;

    // get magnification of orthographic camera
    const double c1 = h(0,0)*h(0,0) + h(1,0)*h(1,0) + h(3,0)*h(3,0) + h(4,0)*h(4,0);
    const double c2 = std::pow((h(0,0)*h(4,0) - h(1,0)*h(3,0)),2);
    Eigen::Matrix<double, 5, 1> coeff;
    // order of coeffs are swapped in Eigen compared to numpy and Matlab
    coeff << c2, 0., -c1, 0., 1;
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    bool has_real_root = false;
    (*magnification) = solver.greatestRealRoot(has_real_root);
    if (!has_real_root || std::isinf((*magnification)) ||std::isnan((*magnification)) || (*magnification) <= 0.0) {
        LOG(INFO) << "Magnification could not be estimated for this view!";
        return false;
    }

    Eigen::Matrix3d K_init;
    K_init << (*magnification), 0, principal_point[0],
              0, (*magnification), principal_point[1],
              0, 0, 1;

    const Eigen::Matrix3d E = PseudoInverse(K_init) * H;

    // get translation
    Eigen::Vector3d t(
            (H(0,2)-principal_point[0]) / (*magnification),
            (H(1,2)-principal_point[1]) / (*magnification),
            0.0);

    // get rotation
    const double term1 = std::pow(E(0,0),2) + std::pow(E(1,0),2);
    const double term2 = std::pow(E(0,1),2) + std::pow(E(1,1),2);
    // Avoid getting imaginary values in sqrt
    if (term1 > 1.0 || term2 > 1.0) {
        return false;
    }
    const double r13 = std::sqrt(1.0 - term1);
    const double r23 = std::sqrt(1.0 - term2);
    const Eigen::Vector3d r1(E(0,0), E(1,0), r13);
    const Eigen::Vector3d r2(E(0,1), E(1,1), r23);
    const Eigen::Vector3d r3 = r1.cross(r2);

    Eigen::Matrix3d Rtmp;
    Rtmp.col(0) = r1;
    Rtmp.col(1) = r2;
    Rtmp.col(2) = r3;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(Rtmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R1 = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();

    solution_translations->push_back(t);
    solution_rotations->push_back(R1);

    // rotation 2 is also a valid solution for planar points
    Eigen::Quaterniond q1(R1);
    Eigen::Quaterniond q2(q1.w(),-q1.x(),-q1.y(),q1.z());
    solution_translations->push_back(t);
    solution_rotations->push_back(q2.matrix());

    return true;
}
}  // namespace theia
