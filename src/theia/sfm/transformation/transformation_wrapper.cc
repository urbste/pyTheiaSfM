#include "theia/sfm/transformation/transformation_wrapper.h"
#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/transformation/align_rotations.h"
#include "theia/sfm/transformation/align_reconstructions.h"
#include "theia/sfm/transformation/gdls_similarity_transform.h"
#include "theia/sfm/transformation/transform_reconstruction.h"

namespace theia {

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> AlignPointCloudsUmeyamaWrapper(const std::vector<Eigen::Vector3d>& left,
                             const std::vector<Eigen::Vector3d>& right){
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double scale;
    AlignPointCloudsUmeyama(left, right, &rotation, &translation, &scale);
    return std::make_tuple(rotation, translation, scale);
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> AlignPointCloudsUmeyamaWithWeightsWrapper(
    const std::vector<Eigen::Vector3d>& left,
    const std::vector<Eigen::Vector3d>& right,
        const std::vector<double>& weights){
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double scale;
    AlignPointCloudsUmeyamaWithWeights(left, right, weights, &rotation, &translation, &scale);
    return std::make_tuple(rotation, translation, scale);
}

std::tuple<std::vector<Eigen::Matrix<double,4,1>>, std::vector<Eigen::Vector3d>, std::vector<double>> GdlsSimilarityTransformWrapper(const std::vector<Eigen::Vector3d>& ray_origin,
                             const std::vector<Eigen::Vector3d>& ray_direction,
                                    const std::vector<Eigen::Vector3d>& world_point){
    std::vector<Eigen::Quaterniond> solution_rotation_q;
    std::vector<Eigen::Vector3d> solution_translation;
    std::vector<double> solution_scale;
    GdlsSimilarityTransform(ray_origin, ray_direction, world_point, &solution_rotation_q, &solution_translation, &solution_scale);


    std::vector<Eigen::Matrix<double,4,1>> solution_rotation;
    for(int i=0;i<solution_rotation_q.size();++i){
        Eigen::Matrix<double,4,1> tmp;
        tmp(0,0) = solution_rotation_q[i].w();
        tmp(1,0) = solution_rotation_q[i].x();
        tmp(2,0) = solution_rotation_q[i].y();
        tmp(3,0) = solution_rotation_q[i].z();
        solution_rotation.push_back(tmp);
    }

    return std::make_tuple(solution_rotation, solution_translation, solution_scale);
}


std::vector<Eigen::Vector3d> AlignRotationsWrapper(const std::vector<Eigen::Vector3d>& gt_rotation){
    std::vector<Eigen::Vector3d> rotation;
    AlignRotations(gt_rotation, &rotation);
    return rotation;
}

Reconstruction AlignReconstructionsWrapper(const Reconstruction& reconstruction1){
    Reconstruction reconstruction2;
    AlignReconstructions(reconstruction1, &reconstruction2);
    return reconstruction2;
}


Reconstruction AlignReconstructionsRobustWrapper(const double robust_error_threshold,
                                                 const Reconstruction& reconstruction1){
    Reconstruction reconstruction2;
    AlignReconstructionsRobust(robust_error_threshold, reconstruction1, &reconstruction2);
    return reconstruction2;
}

Reconstruction TransformReconstructionWrapper(const Eigen::Matrix3d& rotation,
                             const Eigen::Vector3d& translation,
                             const double scale){
    Reconstruction reconstruction;
    TransformReconstruction(rotation, translation, scale, &reconstruction);
    return reconstruction;
}


} //namespace theia
