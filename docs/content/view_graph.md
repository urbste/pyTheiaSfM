# View graph {#documentation-view-graph}

The **view graph** is an **undirected graph** used in Structure from Motion: **vertices** are **`ViewId`**s (images / cameras in your problem), and **edges** carry **pairwise relative geometry** between two views in a **`TwoViewInfo`** (focal lengths, relative rotation, relative position, inlier counts, optional scale cues, etc.).

It is the usual bridge between **feature matching / two-view verification** and **global or incremental pose estimation**: you add an edge for each image pair with a trusted relative pose, then run rotation averaging, translation estimation, filtering, etc., on that graph.

C++ core: [`src/theia/sfm/view_graph/view_graph.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/view_graph/view_graph.h). Pairwise metadata: [`twoview_info.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/twoview_info.h).

## `ViewGraph` API {#view-graph-class}

Vertices are implied by any **`ViewId`** that appears in an edge or that you treat as present (there is no separate “add vertex” call—**`AddEdge`** adds endpoints as needed).

| Method | Meaning |
|--------|---------|
| `AddEdge(view_id_1, view_id_2, two_view_info)` | Add or **update** the undirected edge \((v_1, v_2)\). |
| `RemoveEdge(view_id_1, view_id_2)` | Remove edge; returns whether it existed. |
| `RemoveView(view_id)` | Remove vertex and **all** incident edges. |
| `HasView` / `HasEdge` | Query existence. |
| `NumViews` / `NumEdges` | Counts. |
| `GetNeighborIdsForView(view_id)` | Neighbors (read-only set); `None` if view missing. |
| `GetEdge` / `GetMutableEdge` | Access **`TwoViewInfo`** for an edge. |
| `GetAllEdges` | Map \((\min id,\max id)\rightarrow\) **`TwoViewInfo`** (each edge once). |
| `ReadFromDisk` / `WriteToDisk` | **cereal** serialization of the graph. |

!!! note "Python bindings"

    `ExtractSubgraph` and `GetLargestConnectedComponentIds` are **not** currently exposed in pybind (see `sfm.cc`). You can still prune the graph by removing views/edges from Python or by operating on edge lists you build yourself.

## `TwoViewInfo` {#twoview-info}

Stores **relative pose of view 2 with respect to view 1**, with **view 1 at the origin** and identity rotation by convention:

- `focal_length_1`, `focal_length_2`
- `rotation_2` — angle-axis (Ceres convention) for \(R_2\)
- `position_2` — center of camera 2 in the **coordinate system of camera 1**
- `num_verified_matches`, `num_homography_inliers`, `visibility_score`, `scale_estimate` — bookkeeping for reconstruction heuristics

Use **`EstimateTwoViewInfo`** (and related filters) to populate edges from raw matches; see [Pose](pose.md) and [SfM](sfm.md).

## Graph-level utilities {#view-graph-utilities}

These free functions in **`pytheia.sfm`** operate on a **`ViewGraph`** or on parallel structures:

- **`RelativeRotationsFromViewGraph`** — builds the sparse relative-rotation problem input for global rotation solvers (see [Global pose estimation](global_pose_estimation.md)).
- **`FilterViewGraphCyclesByRotation`** — consistency filtering on the rotation graph.
- **`RemoveDisconnectedViewPairs`** — drops pairs that are not in the main connected structure (parameters in the C++ headers under `view_graph/`).
- **`FilterViewPairsFromOrientation`**, **`FilterViewPairsFromRelativeTranslation`** — thin inconsistent translations / orientations.

Matching and geometric verification that **create** the initial edges are outside this module (Python OpenCV / learning-based matchers, or your own pipeline).

## Typical workflow {#view-graph-workflow}

1. Build a **`ViewGraph`**.
2. For each verified image pair, **`AddEdge`** with a filled **`TwoViewInfo`**.
3. **Filter** edges (cycles, translation, disconnects) as needed.
4. Feed **`RelativeRotationsFromViewGraph`** (or incremental seeding) into [**global pose estimation**](global_pose_estimation.md) or an incremental reconstructor.
5. Produce a **`Reconstruction`**; triangulate and run [**bundle adjustment**](bundle_adjustment.md).

## See also {#view-graph-see-also}

- [SfM](sfm.md) — `ReconstructionEstimator`, view/track types  
- [Global pose estimation](global_pose_estimation.md) — rotation / position from a view graph  
- [Triangulation](triangulation.md) — 3D points from known poses  
