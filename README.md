
pyTheia - A Python Structure-from-Motion and Geometric Vision Swiss Knife
---------------------

pyTheia is based on [TheiaSfM](http://www.theia-sfm.org).
It contains Python bindings for most of the functionalities of TheiaSfM and more.

**The library is still in active development and the interfaces are not yet all fixed**

With pyTheia you have access to a variety of different camera models, structure-from-motion pipelines and geometric vision algorithms.

# Differences to the original library TheiaSfM
pyTheia does not aim at being an end-to-end SfM library. For example, building robust feature detection and matching pipelines is usually application and data specific (e.g. image resolution, runtime, pose priors, invariances, ...). This includes image pre- and postprocessing. 

pyTheia is rather a "swiss knife" for quickly prototyping SfM related reconstruction applications without sacrificing perfomance.
For example SOTA feature detection & matching, place recognition algorithms are based on deep learning, and easily usable from Python. However, using these algorithms from a C++ library is not always straighforward and especially quick testing and prototyping is cumbersome.

## What was removed
Hence, we removed some libaries from the original TheiaSfM:
* SuiteSparse: Optional for ceres, however all GPL related code was removed from src/math/matrix/sparse_cholesky_llt.cc (cholmod -> Eigen::SimplicialLDLT). This will probably be slower on large problems and potentially numerically a bit more unstable.
* OpenImageIO: was used for image in and output and for recitification.
* RapidJSON: Camera intrinsic in and output. Is part of cereal headers anyways.
* RocksDB: Used for saving and loading extracted features efficiently.

## Changes to the original TheiaSfM library


* Global SfM algorithms:
  * LiGT position solver
  * Lagrange Dual rotation estimator
  * Hybrid rotation estimator
  * Possibility to fix multiple views in Robust_L1L2 solver
  * Nonlinear translation solver can fix multiple view or estimate all remaining views in reconstruction
* Camera models
  * Double Sphere
  * Extended Unified
  * Orthographic
* Bundle adjustment
  * Using a homogeneous representation for scene points
  * Extracting covariance information
  * Possibility to add a depth prior to 3D points
  * Position prior for camera poses (e.g. for GPS or known positions)
* General
  * Added timestamp, position_prior_, position_prior_sqrt_information_ variables to **View** class
  Eigen::Matrix3d position_prior_sqrt_information_;
  * Added inverse_depth_, reference_descriptor, reference_bearing_ variables to **Track** class
  * Added covariance_, depth_prior_, depth_prior_variance_ to **Feature** class
* Absolute Pose solvers
  * SQPnP
  * UncalibratedPlanarOrthographic Pose

## Usage Examples

### Full reconstruction example: Global, Hybrid or Incremental SfM using OpenCV feature detection and matching
Have a look at the short example: [sfm_pipeline.py](pytest/sfm_pipeline.py).
Download the south_building dataset from [here](https://demuc.de/colmap/datasets/).
Extract it somewhere and run: 
```bash
python pytest/sfm_pipeline.py --image_path /path/to/south-building/images/
```

### Creating a camera
The following example show you how to create a camera in pyTheia.
You can construct it from a pt.sfm.CameraIntrinsicsPrior() or set all parameters using respective functions from pt.sfm.Camera() class.
``` Python
import pytheia as pt
prior = pt.sfm.CameraIntrinsicsPrior()
prior.focal_length.value = [1000.]
prior.aspect_ratio.value = [1.]
prior.principal_point.value = [500., 500.]
prior.radial_distortion.value = [0., 0., 0., 0]
prior.tangential_distortion.value = [0., 0.]
prior.skew.value = [0]
prior.camera_intrinsics_model_type = 'PINHOLE' 
#'PINHOLE', 'DOUBLE_SPHERE', 'EXTENDED_UNIFIED', 'FISHEYE', 'FOV', 'DIVISION_UNDISTORTION'
camera = pt.sfm.Camera()
camera.SetFromCameraIntrinsicsPriors(prior)

# the camera object also carries extrinsics information
camera.SetPosition([0,0,-2])
camera.SetOrientationFromAngleAxis([0,0,0.1])

# project with intrinsics image to camera coordinates
camera_intrinsics = camera.CameraIntrinsics()
pt2 = [100.,100.]
pt3 = camera_intrinsics.ImageToCameraCoordinates(pt2)
pt2 = camera_intrinsics.CameraToImageCoordinates(pt3)

# project with camera extrinsics
pt3_h = [1,1,2,1] # homogeneous 3d point
depth, pt2 = camera.ProjectPoint(pt3_h)
# get a ray from camera to 3d point in the world frame
ray = camera.PixelToUnitDepthRay(pt2)
pt3_h_ = ray*depth + camera.GetPosition() # == pt3_h[:3]
```

### Solve for absolute or relative camera pose
pyTheia integrates a lot of performant geometric vision algorithms. 
Have a look at the [tests](pytest/sfm)
``` Python
import pytheia as pt

# absolute pose
pose = pt.sfm.PoseFromThreePoints(pts2D, pts3D) # Kneip
pose = pt.sfm.FourPointsPoseFocalLengthRadialDistortion(pts2D, pts3D)
pose = pt.sfm.FourPointPoseAndFocalLength(pts2D, pts3D)
pose = pt.sfm.DlsPnp(pts2D, pts3D)
... and more

# relative pose
pose = pt.sfm.NormalizedEightPointFundamentalMatrix(pts2D, pts2D)
pose = pt.sfm.FourPointHomography(pts2D, pts2D)
pose = pt.sfm.FivePointRelativePose(pts2D, pts2D)
pose = pt.sfm.SevenPointFundamentalMatrix(pts2D, pts2D)
... and more

# ransac estimation
params = pt.solvers.RansacParameters()
params.error_thresh = 0.1
params.max_iterations = 100
params.failure_probability = 0.01

# absolute pose ransac
correspondences2D3D = pt.matching.FeatureCorrespondence2D3D(
  pt.sfm.Feature(point1), pt.sfm.Feature(point2))

pnp_type =  pt.sfm.PnPType.DLS #  pt.sfm.PnPType.SQPnP,  pt.sfm.PnPType.KNEIP
success, abs_ori, summary = pt.sfm.EstimateCalibratedAbsolutePose(
  params, pt.sfm.RansacType(0), pnp_type, correspondences2D3D)

success, abs_ori, summary = pt.sfm.EstimateAbsolutePoseWithKnownOrientation(
  params, pt.sfm.RansacType(0), correspondences2D3D)
... and more
# relative pose ransac
correspondences2D2D = pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(point1), pt.sfm.Feature(point2))

success, rel_ori, summary = pt.sfm.EstimateRelativePose(
        params, pt.sfm.RansacType(0), correspondences2D2D)

success, rad_homog, summary = pt.sfm.EstimateRadialHomographyMatrix(
        params, pt.sfm.RansacType(0), correspondences2D2D)  

success, rad_homog, summary = pt.sfm.EstimateFundamentalMatrix(
        params, pt.sfm.RansacType(0), correspondences2D2D)  
... and more
```

### Bundle Adjustment of views or points
``` Python
import pytheia as pt
recon = pt.sfm.Reconstruction()
# add some views and points
veiw_id = recon.AddView() 
...
track_id = recon.AddTrack()
...
covariance = np.eye(2) * 0.5**2
point = [200,200]
recon.AddObservation(track_id, view_id, pt.sfm.Feature(point, covariance))

# robust BA
opts = pt.sfm.BundleAdjustmentOptions()
opts.robust_loss_width = 1.345
opts.loss_function_type = pt.sfm.LossFunctionType.HUBER

res = BundleAdjustReconstruction(opts, recon)
res = BundleAdjustPartialReconstruction(opts, {view_ids}, {track_ids}, recon)
res = BundleAdjustPartialViewConstant(opts, {var_view_ids}, {const_view_ids}, recon)

# optimize absolute pose on normalized 2D 3D correspondences
res = pt.sfm.OptimizeAbsolutePoseOnNormFeatures(
  [pt.sfm.FeatureCorrespondence2D3D], R_init, p_init, opts)

# bundle camera adjust pose only
res = BundleAdjustView(recon, opts, view_id)
res = BundleAdjustViewWithCov(recon, view_id)
res = BundleAdjustViewsWithCov(recon, opts, [view_id1,view_id2])

# optimize structure only
res = BundleAdjustTrack(recon, opts, trackid)
res = BundleAdjustTrackWithCov(recon, opts, [view_id1,view_id2])
res = BundleAdjustTracksWithCov(recon, opts, [view_id1,trackid])

# two view optimization
res = BundleAdjustTwoViewsAngular(recon, [pt.sfm.FeatureCorrespondence], pt.sfm.TwoViewInfo())
```

## Building
This section describes how to build on Ubuntu locally or on WSL2 both with sudo rights.
The basic dependency is:
* [http://ceres-solver.org/](ceres-solver)

Installing the ceres-solver will also install the neccessary dependencies for pyTheia:
* gflags
* glog
* Eigen

```bash
sudo apt install cmake build-essential 

# cd to your favourite library folder
mkdir LIBS
cd LIBS

# eigen
git clone https://gitlab.com/libeigen/eigen
cd eigen && git checkout 3.4.0
mkdir -p build && cd build && cmake .. && sudo make install

# libgflags libglog libatlas-base-dev
sudo apt install libgflags-dev libgoogle-glog-dev libatlas-base-dev

# ceres solver
cd LIBS
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver && git checkout 2.1.0 && mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF
make -j && make install
```

### Local build without sudo
To build it locally it is best to set the EXPORT_BUILD_DIR flag for the ceres-solver.
You will still need ```sudo apt install libgflags-dev libgoogle-glog-dev libatlas-base-dev```. 
So go ask your admin ;)

```bash
# cd to your favourite library folder. The local installation will be all relative to this path!
mkdir /home/LIBS
cd /home/LIBS

# eigen
git clone https://gitlab.com/libeigen/eigen
cd eigen && git checkout 3.4.0
mkdir -p build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/home/LIBS/eigen/build && make -j install

cd /home/LIBS
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver && git checkout 2.1.0 && mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DEXPORT_BUILD_DIR=ON
make -j

# cd to the pyTheiaSfM folder
cd pyTheiaSfM && mkdir build && cd build 
cmake -DEigen3_DIR=/home/LIBS/eigen/build/share/eigen3/cmake/ .. 
make -j
```

## How to build Python wheels
### Local build with sudo installed ceres-solver and Eigen
Tested on Ubuntu. In your Python >= 3.6 environment of choice run:
```bash
sh build_and_install.sh
```

If you have problems like **/lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found** on Ubuntu 22.04 in an Anaconda environment try:
```bash
conda install -c conda-forge libstdcxx-ng
```

### With Docker
The docker build will actually build manylinux wheels for Linux (Python 3.6-3.12).
There are two ways to do that. One will clutter the source directory, but you will have the wheel file directly available (./wheelhouse/).
Another drawback of this approach is that the files will have been created with docker sudo rights and are diffcult to delete:
```bash
# e.g. for python 3.9
docker run --rm -e PYTHON_VERSION="cp39-cp39" -v `pwd`:/home urbste/pytheia_base:1.2.0 /home/pypackage/build-wheel-linux.sh
```

The other one is cleaner but you will have to copy the wheels out of the docker container afterwards:
```bash
docker build -t pytheia:0.1 .
docker run -it pytheia:0.1
```
Then all the wheels will be inside the container in the folder /home/wheelhouse.
Open a second terminal and run
```bash
docker ps # this will give you a list of running containers to find the correct CONTAINER_ID
docker cp CONTAINER_ID:/home/wheelhouse /path/to/result/folder/pytheia_wheels
```