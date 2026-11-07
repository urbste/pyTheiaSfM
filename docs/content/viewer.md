# Browser viewer {#chapter-viewer}

`pytheia.viewer` turns a `pytheia.sfm.Reconstruction` into an interactive 3D page. The page is a single HTML file with three.js r160, its OrbitControls and the scene data inlined, so it needs no server and no network. The package is pure Python and needs only numpy.

```python
import pytheia as pt
from pytheia import viewer

rec = pt.sfm.Reconstruction()
# ... build or load the reconstruction ...

viewer.export_html(rec, "scene.html")          # open the file in a browser
viewer.show(rec, height=600)                   # notebook: IPython.display.HTML iframe
server = viewer.serve(rec, port=8000)          # http://127.0.0.1:8000/ until Ctrl-C

other = pt.sfm.Reconstruction()                # e.g. a second run or a GPS-scaled model
viewer.compare(rec, other, "compare.html")     # other is aligned onto rec (Sim(3))
```

## API

Optional arguments are keyword arguments with defaults; pass them by name.

| Function | Returns | Notes |
|----------|---------|-------|
| `export_html(reconstruction, path, *, max_points=None, thumbnails_dir=None, include_view_graph=None, compare_with=None, title=..., theme="dark")` | absolute path of the written file | One self-contained `.html` file. |
| `compare(reconstruction, other, path, **kwargs)` | absolute path of the written file | `export_html` with `compare_with=other`. Keyword arguments as in `export_html`. |
| `show(reconstruction, height=600, *, max_points=..., thumbnails_dir=..., include_view_graph=..., compare_with=..., title=..., theme=...)` | `IPython.display.HTML` with an iframe (`srcdoc`) | IPython is imported lazily. Nothing is written to disk. |
| `serve(reconstruction, port=8000, *, host="127.0.0.1", block=True, ...)` | the `ThreadingHTTPServer` | `block=False` serves from a daemon thread; call `server.shutdown()` to stop. `port=0` picks a free port. |
| `build_html(reconstruction, *, ...)` | the HTML as `str` | What the other functions write. |
| `recon_to_scene(reconstruction, max_points=None, thumbnails_dir=None, include_view_graph=None, *, frustum_scale=None, seed=0, compare_with=None)` | `dict` of numpy arrays | The data layer; see below. |
| `scene_to_payload(scene)` | JSON-ready `dict` | Arrays as base64 buffers. |
| `decode_array(desc)` | numpy array | Inverse of one payload array. |
| `align_scene(scene_a, scene_b, *, min_shared=3)` | `dict` or `None` | Sim(3) alignment of two scenes on shared views (see Compare). |
| `umeyama_sim3(src, dst)` | `(scale, R, t)` | Least-squares similarity `dst ~= scale * R @ src + t`. |

Options:

- `max_points`: keep a random subset of the estimated points (fixed `seed`). Observations and per-view statistics refer to the kept points.
- `thumbnails_dir`: a folder with one image per view. Files are matched by view name (relative path first, then file stem, any of jpg, jpeg, png, webp, gif, bmp). They are embedded as data URIs, so resize them first (for example to 256 px) to keep the file small.
- `include_view_graph`: adds covisibility edges between registered views. The weight is the number of kept points seen by both views. pyTheia does not store its two-view view graph in the reconstruction, so the edges come from shared tracks. The edges are off by default and can be toggled in the page.
- `compare_with`: a second reconstruction, drawn next to the first. See Compare below.
- `theme`: `"dark"` or `"light"` initially; the page has a toggle.

## Compare

`viewer.compare(a, b, path)` (or `export_html(a, path, compare_with=b)`) aligns `b` onto `a`:

1. The views that are registered in both reconstructions **and have the same view name** are the shared views. Their camera centers are the correspondences.
2. A least-squares similarity transform (scale, rotation, translation; Umeyama, 1991) is fitted: `center_a ~= scale * R @ center_b + t`. The rotation is proper (det +1).
3. All of `b` (points, cameras, frusta, view graph) is transformed into the frame of `a`. Camera rotations become `R_w2c @ R^T`; observations and reprojection errors do not change.

What you get in the page:

- Two sets, `A` (the first reconstruction, RGB points, blue cameras) and `B (aligned)` (magenta points and cameras). Each set has its own checkbox. Unregistered cameras stay orange in both.
- The summary lists the number of shared views, the fitted scale and the RMS center error after alignment, in scene units of `a`.
- The view table of `A` gets two columns: **center err** (distance between the camera center in `A` and the aligned center in `B`, scene units) and **rot deg** (angle between the two orientations after alignment). Clicking a camera shows its matched view in `B` and the same errors.

Limits of the comparison:

- The fit uses camera centers only and is a least-squares fit, so one badly placed camera pulls the whole alignment. Read the RMS and the per-camera errors before trusting a difference.
- With fewer than 3 shared views (or collinear camera centers) no alignment is possible. A `UserWarning` is issued and only `a` is shown. The page is still written.
- The per-point comparison is not a point-to-point match. Points of `B` are drawn as they are, after the transform.
- Only `a` is the reference frame. Its scale and axes are kept; `b` is mapped into them.

## What the page does

- **Orbit, pan, zoom** with the mouse (OrbitControls, the default). Reset view and center-on-selection buttons.
- **Fly mode** (checkbox "fly mode (WASD)"): first-person navigation. `W`/`S` move forward and back, `A`/`D` strafe, `Space`/`E` go up, `Shift`/`Q` go down. Drag with the left mouse button to look around. The mouse wheel changes the flight speed. Unchecking the box returns to orbit around the point in front of the camera.
- **Points**: point size, colors by RGB, track length (log scale), mean reprojection error (99th percentile range; gray for points without error), or **distance** from the selected camera (or from the scene centroid when nothing or a point is selected). The distance range is 0 to the 99th percentile of the distances.
- **Filters**: minimum track length and maximum reprojection error (px). The count of shown points is displayed.
- **Cameras**: centers and image-plane frusta. Registered views are blue. Unregistered views are orange (toggle "highlight unregistered"). The image-plane depth is 5 % of the point bounding-box diagonal unless `recon_to_scene(..., frustum_scale=...)` is given.
- **Trajectory** (checkbox "trajectory"): a line through the registered cameras, ordered by view name (natural order, so `img2` comes before `img10`) or by view id (select next to the checkbox).
- **Measure** (checkbox "measure (2 clicks)"): click two cameras or points. The page shows the distance in scene units and draws the segment. Click again to start a new measurement; "Clear" removes it. In compare mode the distance is in the frame of `A`.
- **Click a camera** (within 12 px of its center): name, view id, registration state, image size, intrinsics, center, number of observed points (drawn in yellow), mean error, the thumbnail if one was given, and in compare mode the matched view and the errors.
- **Click a point** (within 6 px): track id, position, color, track length, mean error, and a table of the views that observe it with pixel positions and residuals. Clicking a row selects that camera.
- **Views table**: all views of the first reconstruction with registration state, observation count and mean error (plus the compare errors). Click a header to sort; click a row to select the camera.
- **Dark and light theme**, and **Screenshot (PNG)** of the current canvas.

Picking uses screen-space distances, so it does not depend on the point size.

## Scene data

`recon_to_scene` returns:

| Key | Arrays (numpy) | Notes |
|-----|----------------|-------|
| `points` | `xyz` (N,3) f32, `rgb` (N,3) u8, `track_length` (N,) f32, `reprojection_error` (N,) f32, `track_id` (N,) f32, `obs_offsets` (N+1,) f32 | Estimated tracks with a finite point. Error = mean pixel residual over the observations in estimated views; NaN when there are none. `track_id` is stored as float32 and is exact below 2^24. |
| `observations` | `view` (M,) f32, `x`, `y`, `residual` (M,) f32 | Grouped by point. Point `i` owns `[obs_offsets[i], obs_offsets[i+1])`. `view` is the camera index (position in `cameras`), not the view id. |
| `cameras` | `names` (list), `view_id` (V,), `estimated` (V,) u8, `center` (V,3), `rotation_w2c` (V,3,3), `intrinsics` (V,4) as fx, fy, cx, cy, `image_size` (V,2), `frustum` (V,4,3), `num_observations` (V,), `mean_reprojection_error` (V,), `thumbnails` (list of data URI or None) | `center` is `Camera.GetPosition()`, `rotation_w2c` is `Camera.GetOrientationAsRotationMatrix()`, and intrinsics come from `Camera.GetCalibrationMatrix()`. |
| `edges` | `pairs` (E,2) f32, `weight` (E,) f32, or `None` | Only with `include_view_graph`. |
| `compare` | dict or `None` | Only with `compare_with` and an alignment. `scene` is the aligned second scene (same layout), `sim3` (`scale`, `rotation`, `translation`), `num_shared`, `rms_center_error`, and `cameras` with `matched_b` (index into the second scene's cameras, -1 if none), `center_error` and `rotation_error_deg` (V,, NaN if none). |
| `stats`, `bounds` | dicts | Counts, mean error, bounding box and frustum depth. |

Conventions:

- Coordinates are the reconstruction's world frame and are not transformed. The viewer uses +Y as up, so a reconstruction whose world is y-down looks upside down; rotate it before export if that matters. Camera axes follow OpenCV (x right, y down, z forward), as in `Camera.ProjectPoint`.
- `x_cam = R (X - C)` and `pixel = K x_cam / z`, with `R = rotation_w2c` and `C = center`. The camera frustum corners are `C + R^T K^-1 (u, v, 1) d`.
- Distorted camera models are drawn with their pinhole approximation (`GetCalibrationMatrix`). Observations and residuals are not undistorted.

Binary buffers are base64 of little-endian `float32` or `uint8` arrays, with `{"dtype", "shape", "data"}` descriptors in the page's JSON (`<script id="pytheia-scene" type="application/json">`). `decode_array` reads them back. A headless check is shown in `pytests/viewer/test_viewer.py`.

## Offline use and vendored files

The package ships these files in `pytheia/viewer/assets/`:

| File | Source |
|------|--------|
| `three.module.min.js` | `https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js` |
| `OrbitControls.js` | `https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js` |
| `viewer.js`, `viewer.html` | pyTheia |

Both three.js files keep their MIT license headers. The page loads them from base64 text through blob URLs, and the `from 'three'` import in OrbitControls is rewritten to the blob of the three.js module. This avoids any network request. `compare.py` and the Sim(3) fit are numpy only.

## Limits

- The page renders all shown points of each set in one draw call. Millions of points are feasible but slow to pick; use `max_points` for very large reconstructions. Compare mode doubles the number of points in the file.
- Thumbnails are not resized by pyTheia. Pre-resize them.
- The fly mode uses mouse drag for looking; pointer lock is not used.
- The compare alignment is not a robust estimator (no outlier rejection). Check the RMS error and the per-camera errors.
- Verification: the data layer, the Sim(3) fit and alignment, the embedded JSON, the compare payload and the HTTP server are covered by pytest. In headless Chrome (swiftshader, `file://`) on synthetic scenes the following were checked by driving the page: boot in single and compare mode, the compare summary and error columns, toggling sets, depth coloring with camera and centroid references, trajectory with both orders, measure distances, fly-mode movement, mouse look and return to orbit, and the fallback page for fewer than 3 shared views. Not verified: real mouse behaviour on your own data, the screenshot button, fly mode on large scenes, and notebook display (`show()`).
- The page exposes `window.pytheia.select('cam' | 'point', index, set = 0)`, `screenPos(type, set, index)`, `setFly(on)` and `state` for scripting and debugging.
