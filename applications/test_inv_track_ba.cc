
#include <theia/theia.h>

using namespace theia;

    RandomNumberGenerator rng(52);

    Camera RandomCamera() {
    Camera camera;
    camera.SetPosition(rng.RandVector3d());
    camera.SetOrientationFromAngleAxis(0.2 * rng.RandVector3d());
    camera.SetImageSize(1000, 1000);
    camera.SetFocalLength(500);
    camera.SetPrincipalPoint(500, 500);
    return camera;
    }
Camera FixCamera(const Eigen::Vector3d& position) {
  Camera camera;
  camera.SetPosition(position);
  camera.SetOrientationFromAngleAxis(0.001 * rng.RandVector3d());
  camera.SetImageSize(1000, 1000);
  camera.SetFocalLength(500);
  camera.SetPrincipalPoint(500, 500);
  return camera;
}

void TestOptimizeTracks(const int kNumPoints, const double kPixelNoise, 
  const TrackParametrizationType& track_type) {
  // Set up random cameras.
  Camera camera1 = FixCamera(Eigen::Vector3d(0.0,0.0,0.0));
  Camera camera2 = FixCamera(Eigen::Vector3d(2.0,0.0,0.0));
  Camera camera3 = FixCamera(Eigen::Vector3d(-2.0,0.0,0.0));
  Reconstruction reconstruction;
  ViewId vid1 = reconstruction.AddView("0", 0, 0.0);
  ViewId vid2 = reconstruction.AddView("1", 0, 1.0);
  ViewId vid3 = reconstruction.AddView("2", 0, 2.0);
  reconstruction.MutableView(vid1)->MutableCamera()->DeepCopy(camera1);
  reconstruction.MutableView(vid1)->SetEstimated(true);
  reconstruction.MutableView(vid2)->MutableCamera()->DeepCopy(camera2);
  reconstruction.MutableView(vid2)->SetEstimated(true);
  reconstruction.MutableView(vid3)->MutableCamera()->DeepCopy(camera3);
  reconstruction.MutableView(vid3)->SetEstimated(true);

  // Set up random points.
  for (int i = 0; i < kNumPoints; i++) {
    Eigen::Vector3d point(rng.RandDouble(-5.0, 5.0),
                          rng.RandDouble(-5.0, 5.0),
                          rng.RandDouble(4.0, 10.0));
    TrackId tid = reconstruction.AddTrack();
    const auto track = reconstruction.MutableTrack(tid);
    track->SetPoint(point.homogeneous());

    Eigen::Vector2d pixel1, pixel2, pixel3;
    double depth1 = reconstruction.View(vid1)->Camera().ProjectPoint(
        point.homogeneous(), &pixel1);
    double depth2 = reconstruction.View(vid2)->Camera().ProjectPoint(
        point.homogeneous(), &pixel2);
    double depth3 = reconstruction.View(vid3)->Camera().ProjectPoint(
        point.homogeneous(), &pixel3);    
    if (kPixelNoise > 0.0) {
      AddNoiseToProjection(kPixelNoise, &rng, &pixel1);
      AddNoiseToProjection(kPixelNoise, &rng, &pixel2);
      AddNoiseToProjection(kPixelNoise, &rng, &pixel3);

    }
    if (depth1 > 0.0 && depth2 > 0.0 && depth3 > 0.0) {
      reconstruction.AddObservation(vid1, tid, Feature(pixel1));
      reconstruction.AddObservation(vid2, tid, Feature(pixel2));
      reconstruction.AddObservation(vid3, tid, Feature(pixel3));
      track->SetEstimated(true);

      Eigen::Vector2d tmp;
      const ViewId ref_view_id = track->ReferenceViewId();
      const View* ref_view = reconstruction.View(ref_view_id);
      const Camera ref_cam = ref_view->Camera();
      const double depth = ref_cam.ProjectPoint(point.homogeneous(), &tmp);
      const Feature* reference_obs = ref_view->GetFeature(tid);
      const Eigen::Vector3d bearing_vector = ref_cam.PixelToNormalizedCoordinates(reference_obs->point_);
      track->SetReferenceBearingVector(bearing_vector);
      track->SetInverseDepth(1.0/depth);
    }
  }

  BundleAdjustmentOptions opts;
  if (track_type == TrackParametrizationType::XYZW) {
    opts.use_inverse_depth_parametrization = false;
    opts.use_homogeneous_point_parametrization = false;
  }else if (track_type == TrackParametrizationType::INVERSE_DEPTH) {
    opts.use_inverse_depth_parametrization = true;
    opts.use_homogeneous_point_parametrization = false;
  } else if(track_type == TrackParametrizationType::XYZW_MANIFOLD) {
    opts.use_inverse_depth_parametrization = false;
    opts.use_homogeneous_point_parametrization = true;
  } else {
    LOG(FATAL) << "Unknown track parametrization type.";
  }

  opts.verbose = true;
  // print track inverse depth
  for (const auto& track_id : reconstruction.TrackIds()) {
    Track* track = CHECK_NOTNULL(reconstruction.MutableTrack(track_id));
    if (track == nullptr || !track->IsEstimated()) {
      continue;
    }
    std::cout<<"Old inv depth: "<<track->InverseDepth()<<std::endl;
  }

  BundleAdjustmentSummary sum = BundleAdjustTracks(opts, 
    reconstruction.TrackIds(), &reconstruction);
  // print track inverse depth
  for (const auto& track_id : reconstruction.TrackIds()) {
    Track* track = CHECK_NOTNULL(reconstruction.MutableTrack(track_id));
    if (track == nullptr || !track->IsEstimated()) {
      continue;
    }
    std::cout<<"New inv depth: "<<track->InverseDepth()<<std::endl;
  }
  
  size_t num_obs = 0;
  for (auto v_id : reconstruction.ViewIds()) {
    num_obs += reconstruction.View(v_id)->NumFeatures();
  }

  std::cout << "Success: " << sum.success << "\n";
  std::cout << "Final squared reprojection error: " << 2.0 * sum.final_cost / num_obs
            << "\n";
  }

int main(int argc, char** argv) {

  TestOptimizeTracks(10, 0.5, TrackParametrizationType::XYZW_MANIFOLD);
  TestOptimizeTracks(10, 0.5, TrackParametrizationType::XYZW);
  TestOptimizeTracks(10, 0.5, TrackParametrizationType::INVERSE_DEPTH);
  
  return 0;
}