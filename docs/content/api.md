# API Reference {#chapter-api}

The chapters below document the underlying **C++** Theia library (classes, functions, and conventions). pyTheia exposes the same concepts through Python bindings; see [Python API overview](python_wrapper.md) for import layout and stubs. For runnable scripts see [Examples showcase](examples_showcase.md).

- [Image](image.md) — scope of image handling in pyTheia (Python / OpenCV)  
- [Ransac](ransac.md) — robust estimation interfaces  
- [Pose](pose.md) — absolute and relative pose / resectioning  
- [Estimators](estimators.md) — RANSAC-wrapped pose and two-view estimators (`src/theia/sfm/estimators`)  
- [Global pose estimation](global_pose_estimation.md) — rotation averaging and global positions (`src/theia/sfm/global_pose_estimation`)  
- [Math](math.md) — math utilities and solvers  
- [Cameras](cameras.md) — camera models and projection  
- [SfM](sfm.md) — reconstruction, pipelines, core data structures  
- [Triangulation](triangulation.md) — multi-view point estimation and robust triangulation  
- [View graph](view_graph.md) — image pairs, `TwoViewInfo`, graph utilities  
- [Matching](matching.md) — correspondences, match database, `FeatureMatcherOptions` (`src/pytheia/matching`)  
- [MVS](mvs.md) — view selection for MVSNet-style pipelines (`pytheia.mvs`)  
- [Bundle adjustment](bundle_adjustment.md) — Ceres-based joint refinement  
- [Transformations](transformations.md) — Sim3 / Umeyama alignment, gDLS, transform reconstructions  
- [IO](io.md) — reading and writing reconstruction data  
