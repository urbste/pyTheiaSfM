
// Author: Steffen Urban (urbste@gmail.com)

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <theia/theia.h>

#include <algorithm>
#include <memory>
#include <string>

#include "print_reconstruction_statistics.h"

DEFINE_string(reconstruction, "", "Reconstruction file");

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Load the SIFT descriptors into the cameras.
  std::unique_ptr<theia::Reconstruction> reconstruction(
      new theia::Reconstruction());
  CHECK(theia::ReadReconstruction(FLAGS_reconstruction, reconstruction.get()))
      << "Could not read reconstruction file.";

  std::cout << "\nNum views: " << reconstruction->NumViews()
            << "\nNum 3D points: " << reconstruction->NumTracks() << "\n";

  theia::BundleAdjustmentOptions options;
  for (int i = 0; i < reconstruction->TrackIds().size(); ++i) {
    Eigen::Matrix3d cov;
    double empirical_variance;
    theia::BundleAdjustmentSummary summary =
        theia::BundleAdjustTrack(options,
                                 reconstruction->TrackIds()[i],
                                 reconstruction.get(),
                                 &cov,
                                 &empirical_variance);

    std::cout << "FInal cost: " << summary.final_cost << "\n";
    std::cout << "standard deviations: "
              << cov.diagonal().array().sqrt().transpose() * 1000. << " [mm]\n";
  }
  return 0;
}
