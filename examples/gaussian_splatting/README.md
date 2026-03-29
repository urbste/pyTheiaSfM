# Reconstruction export for 3D Gaussian splatting

pyTheia does **not** train Gaussian splat models inside this repository. After you have a **`Reconstruction`** (binary `.recon` or from memory), you can export camera poses and image paths in **Nerfstudio** format for downstream trainers.

## Quick export

From the repository root (with `pytheia` installed):

```bash
python examples/gaussian_splatting/export_nerfstudio.py \
  --path_to_images /path/to/images \
  --path_to_recon /path/to/reconstruction.recon \
  --path_out_json /path/to/scene/transforms.json \
  --aabb_scale 16
```

The sibling script [`pyexamples/nerfstudio_export_reconstruction.py`](../../pyexamples/nerfstudio_export_reconstruction.py) is the same operation in one file; this wrapper lives next to splat-specific documentation.

## Next steps

See [`TRAIN_SPLAT.md`](TRAIN_SPLAT.md) for **Nerfstudio Splatfacto** and pointers to other 3DGS codebases.

## Supported cameras

Export follows [`WriteNerfStudio`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/write_nerfstudio.h) mappings (**PINHOLE** and **FISHEYE**). Undistorted pinhole imagery may be required by some splat trainers—preprocess externally if needed.
