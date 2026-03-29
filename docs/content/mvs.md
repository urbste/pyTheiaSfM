# Multi-view stereo helpers (`pytheia.mvs`) {#documentation-mvs}

pyTheia exposes a **small** MVS-oriented API from the C++ Theia tree. Bindings: [`mvs.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/pytheia/mvs/mvs.cc).

---

## `ViewSelectionMVSNet`

**`pt.mvs.ViewSelectionMVSNet(reconstruction, num_neighbors, theta0=5.0, sigma1=1.0, sigma2=10.0)`**

Selects, for each view, a ranked list of **neighbor views** suited for **MVSNet-style** depth inference, using **covisibility** and **triangulation angle** cues as in:

> Yao et al., *MVSNet: Depth Inference for Unstructured Multi-view Stereo*, ECCV 2018.

**Returns:** `dict[int, dict[float, int]]` — outer key is a **view id**; each inner dict maps a **score** (float) to a **neighbor view id** (int), with scores ordered for **best-first** iteration (C++ `std::map<double, ViewId, std::greater<double>>`).

**Parameters:**

- **`num_neighbors`** — how many neighbors to retain per view (interpreted with the scoring function in C++).
- **`theta0`**, **`sigma1`**, **`sigma2`** — angular / weighting parameters (see [`view_selection_mvsnet.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/mvs/view_selection_mvsnet.h) and the `.cc` implementation).

This does **not** run a learned MVS network; it only **proposes view subsets** given an existing **`Reconstruction`** with cameras and tracks.

---

## See also

- [SfM](sfm.md) — building a `Reconstruction` before view selection  
- [Triangulation](triangulation.md) — structure needed for meaningful covisibility  
