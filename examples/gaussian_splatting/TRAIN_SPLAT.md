# Training 3D Gaussian splats from pyTheia exports

After [`export_nerfstudio.py`](export_nerfstudio.py) (or [`pyexamples/nerfstudio_export_reconstruction.py`](../../pyexamples/nerfstudio_export_reconstruction.py)) produces `transforms.json` and your images are in the layout Nerfstudio expects, you can train with an external project.

## Nerfstudio — Splatfacto

Install [Nerfstudio](https://docs.nerf.studio/) in a separate environment (versions change quickly; follow their docs).

Typical pattern:

```bash
ns-train splatfacto --data /path/to/scene
```

Point `--data` at the directory containing `transforms.json` and the image tree referenced inside it. Confirm [data conventions](https://docs.nerf.studio/quickstart/custom_dataset.html) match your export.

## Other stacks

- **gsplat** / **nerfacc**-style trainers often expect **COLMAP** text or **Blender / transforms.json**; you can try the same `transforms.json` if the trainer documents compatibility.
- **Original 3D Gaussian Splatting** ([graphdeco-inria/gaussian-splatting](https://github.com/graphdeco-inria/gaussian-splatting)) commonly uses **COLMAP**; pyTheia can write COLMAP text via **`pt.io.WriteColmapFiles(reconstruction, output_directory)`** (see the [IO](https://urbste.github.io/pyTheiaSfM/io/) chapter in the published manual, or `docs/content/io.md` in the repo).

## Practical tips

- Start from a **bundle-adjusted** reconstruction with reasonable reprojection errors.
- **Splatfacto** and many splat methods assume **pinhole** images; heavily distorted fisheye exports may need undistortion in Python/OpenCV before training.
- Match **image resolution** and **principal point** between training and the poses in `transforms.json`.
