#include <Eigen/Core>
#include <string>
#include <vector>

#include "theia/image/keypoint_detector/keypoint.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/view_graph/view_graph.h"

namespace theia {
class Keypoint;
class Reconstruction;
class ViewGraph;


std::tuple<bool, Reconstruction> ImportNVMFileWrapper(const std::string& nvm_filepath);
std::tuple<bool, Reconstruction> PopulateImageSizesAndPrincipalPointsWrapper(const std::string& image_directory);
std::tuple<bool, Reconstruction, ViewGraph> Read1DSFMWrapper(const std::string& dataset_directory);
std::tuple<bool, Reconstruction> ReadBundlerFilesWrapper(const std::string& lists_file, const std::string& bundle_file);
std::tuple<bool, std::vector<Keypoint>, std::vector<Eigen::VectorXf>> ReadKeypointsAndDescriptorsWrapper(const std::string& features_file);
std::tuple<bool, Reconstruction> ReadStrechaDatasetWrapper(const std::string& dataset_directory);
std::tuple<bool, Reconstruction> ReadReconstructionWrapper(const std::string& input_file);
std::tuple<bool, std::vector<Eigen::VectorXf>, std::vector<Keypoint>> ReadSiftKeyBinaryFileWrapper(const std::string& input_sift_key_file);
std::tuple<bool, std::vector<Eigen::VectorXf>, std::vector<Keypoint>> ReadSiftKeyTextFileWrapper(const std::string& sift_key_file);


}  // namespace theia
