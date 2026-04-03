
pyTheia - A Python Structure-from-Motion and Geometric Vision Swiss Knife
---------------------

pyTheia is based on [TheiaSfM](http://www.theia-sfm.org).
It contains Python bindings for most of the functionalities of TheiaSfM and more.

**Documentation:** [https://urbste.github.io/pyTheiaSfM/](https://urbste.github.io/pyTheiaSfM/) (MkDocs; build locally with `pip install -r docs/requirements.txt` and `mkdocs serve -f docs/mkdocs.yml`).

**The library is still in active development and the interfaces are not yet all fixed**

With pyTheia you have access to a variety of different camera models, structure-from-motion pipelines and geometric vision algorithms.

# Differences to the original library TheiaSfM
pyTheia does not aim at being an end-to-end SfM library. For example, building robust feature detection and matching pipelines is usually application and data specific (e.g. image resolution, runtime, pose priors, invariances, ...). This includes image pre- and postprocessing. 

pyTheia is rather a "swiss knife" for quickly prototyping SfM related reconstruction applications without sacrificing perfomance.
For example SOTA feature detection & matching, place recognition algorithms are based on deep learning, and easily usable from Python. However, using these algorithms from a C++ library is not always straighforward and especially quick testing and prototyping is cumbersome.

## Dependency changes
Compared to the original TheiaSfM:
* SuiteSparse: Optional for Ceres; GPL-dependent code was removed in src/math/matrix/sparse_cholesky_llt.cc (cholmod -> Eigen::SimplicialLDLT), which may be slower for very large problems and slightly less stable numerically.
* RapidJSON: No separate dependency; RapidJSON is vendored via cereal headers.
* OpenImageIO / `theia/image`: Not used. Raster images and EXIF are handled in Python (OpenCV, Pillow, etc.); C++ focuses on geometry, matching structures, and SfM pipelines once correspondences exist.

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
Have a look at the short example: [sfm_pipeline.py](pytests/sfm_pipeline.py).
Download the south_building dataset from [here](https://demuc.de/colmap/datasets/).
Extract it somewhere and run: 
```bash
python pytests/sfm_pipeline.py --image_path /path/to/south-building/images/
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
Have a look at the [tests](pytests/sfm)
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
res = BundleAdjustPartialViewsConstant(opts, {var_view_ids}, {const_view_ids}, recon)

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

### Export to Nerfstudio and SDFStudio
You can export a `pt.sfm.Reconstruction` to Nerfstudio or SDFStudio formats directly from Python:
```python
import pytheia as pt
# Nerfstudio (writes transforms.json)
pt.io.WriteNerfStudio("/path/to/images", recon, 16, "/path/to/out/transforms.json")
# SDFStudio (all images must be undistorted)
pt.io.WriteSdfStudio("/path/to/images", recon, (2.0, 6.0), 1.0)
```
More complete examples are in `pyexamples/nerfstudio_export_reconstruction.py` and `pyexamples/sdfstudio_export_reconstruction.py`.

## Building
This section describes how to build on Ubuntu locally or on WSL2 (with sudo where noted).

**Core dependency:** [Ceres Solver](http://ceres-solver.org/) (non-linear least squares for bundle adjustment and many solvers). A normal Ceres install also pulls in **Eigen**, **glog**, and **gflags** (or your distro equivalents).

- Use a **current Ceres 2.x** release (see [Ceres installation](http://ceres-solver.org/installation.html)). Older **2.1.x** is still fine for CPU-only builds.
- **Optional — GPU solvers in Ceres:** build Ceres with **`USE_CUDA=ON`** for **CUDA dense** linear algebra. For **CUDA sparse** (`CUDA_SPARSE` in Ceres), build Ceres with **NVIDIA cuDSS** support so the library includes the **cuDSS** component (details in upstream docs). When you configure **pyTheia**, CMake detects dense CUDA (compile check) and sparse CUDA (`CERES_COMPILED_COMPONENTS`); you can override with `-DTHEIA_CERES_USE_CUDA` / `-DTHEIA_CERES_USE_CUDA_SPARSE` if needed. Runtime selection of GPU backends is via **`BundleAdjustmentOptions`** (`dense_linear_algebra_library_type`, `sparse_linear_algebra_library_type`); see the MkDocs chapter **Bundle adjustment**. Pre-built **manylinux** wheels typically link a **CPU** Ceres — use a **local** build against GPU-enabled Ceres for CUDA backends.

### Example: system install (sudo)

```bash
sudo apt install cmake build-essential libgflags-dev libgoogle-glog-dev libatlas-base-dev

mkdir LIBS && cd LIBS

# Eigen (example 3.4.x)
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && git checkout 3.4.0
mkdir -p build && cd build && cmake .. && sudo cmake --install .

# Ceres — latest 2.x tag; add -DUSE_CUDA=ON / cuDSS CMake variables per Ceres docs for GPU
cd ../..
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver && git fetch --tags && git checkout "$(git tag -l '2.*' | sort -V | tail -1)"
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF
cmake --build . -j"$(nproc)"
sudo cmake --install .
```

### Local build without sudo
Prefer Ceres **`EXPORT_BUILD_DIR=ON`** so `find_package(Ceres)` can use the build tree. You still need development packages for gflags/glog/atlas (or ask your admin).

```bash
mkdir /home/LIBS && cd /home/LIBS

git clone https://gitlab.com/libeigen/eigen.git
cd eigen && git checkout 3.4.0
mkdir -p build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/home/LIBS/eigen/build && cmake --build . -j"$(nproc)" && cmake --install .

cd /home/LIBS
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver && git fetch --tags && git checkout "$(git tag -l '2.*' | sort -V | tail -1)"
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DEXPORT_BUILD_DIR=ON
cmake --build . -j"$(nproc)"

cd /path/to/pyTheiaSfM && mkdir build && cd build
cmake -DEigen3_DIR=/home/LIBS/eigen/build/share/eigen3/cmake/ -DCeres_DIR=/home/LIBS/ceres-solver/build ../
cmake --build . -j"$(nproc)"
```

Full narrative (system deps, CUDA notes, docs build): **`docs/content/building.md`** (also rendered as **Building** on the project docs site).

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

Another solution is to check the GLIBCXX versions. If the version that the library requires is installed, then we can create a symbolic link into the conda environment.
```
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
# if the GLIBCXX version is available then do:
ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 ${CONDA_PREFIX}/lib/libstdc++.so.6
```

### With Docker
The docker build will actually build manylinux wheels for Linux (Python 3.6-3.12).
There are two ways to do that. One will clutter the source directory, but you will have the wheel file directly available (./wheelhouse/).
Another drawback of this approach is that the files will have been created with docker sudo rights and are diffcult to delete:
```bash
# e.g. for python 3.9
docker run --rm -e PYTHON_VERSION="cp39-cp39" -v `pwd`:/home urbste/pytheia_base:1.4.0 /home/pypackage/build-wheel-linux.sh
```

The other one is cleaner but you will have to copy the wheels out of the docker container afterwards:
```bash
docker build -t pytheia:1.0 .
docker run -it pytheia:1.0
```
Then all the wheels will be inside the container in the folder /home/wheelhouse.
Open a second terminal and run
```bash
docker ps # this will give you a list of running containers to find the correct CONTAINER_ID
docker cp CONTAINER_ID:/home/wheelhouse /path/to/result/folder/pytheia_wheels
```

## Typing and editor stubs
To get full function/argument lists and IntelliSense in editors for the native extension:

- Generate stubs locally (requires `pybind11-stubgen`):
   
   ```bash
   pip install pybind11-stubgen
   dev/generate_stubs.sh
   ```
   
   This writes `.pyi` files to `typings/pytheia`. VS Code/Pylance will pick them up via `pyrightconfig.json`.
 
 - When building wheels via `setup.py`, stubs are generated automatically by default. To skip:
   
   ```bash
   GENERATE_STUBS=0 python setup.py bdist_wheel --plat-name=...
   ```
 
 - The package ships a PEP 561 marker (`py.typed`) so downstream type checkers can consume the bundled stubs.