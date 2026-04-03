# Image types {#documentation-image}

The upstream Theia library provided a `FloatImage` class built on [OpenImageIO](https://openimageio.org/) for loading images and driving the built-in SIFT/AKAZE feature pipeline.

**pyTheia removes `src/theia/image/` and does not link OpenImageIO.** There is no C++ raster image type in this fork.

- Load and preprocess images in **Python** (for example OpenCV, Pillow, or PyTorch).
- Do feature extraction and matching in Python (or any external tool), then pass 2D correspondences into pyTheia (`FeatureCorrespondence`, view graphs, incremental/global estimators, etc.).
- Raster **undistortion** of pixel grids is not part of this tree; apply `cv2.undistort` (or similar) in Python if you need corrected images. Adjust intrinsics and feature coordinates yourself if you need a pinhole-consistent representation across the reconstruction.

For historical reference, the old `FloatImage` API lived in upstream [`image.h`](https://github.com/sweeneychris/TheiaSfM/blob/master/src/theia/image/image.h).
