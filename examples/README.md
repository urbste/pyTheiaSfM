# pyTheia showcase examples

Optional, dependency-heavy workflows built around **pyTheia** Structure-from-Motion. For minimal OpenCV-based scripts, see [`pyexamples/`](../pyexamples/) in the repository root.

## Contents

| Directory | Purpose |
|-----------|---------|
| [`vismatch_sfm/`](vismatch_sfm/) | Image pairs matched with [**vismatch**](https://github.com/gmberton/vismatch) (50+ matchers), geometric verification with **pyTheia**, incremental/global/hybrid reconstruction, optional [**Rerun**](https://rerun.io/) visualization. |
| [`gaussian_splatting/`](gaussian_splatting/) | Export a pyTheia reconstruction to **Nerfstudio** `transforms.json` and notes for training **Splatfacto** / other 3D Gaussian splatting pipelines. |

## Setup

Install pyTheia from the repo (see [Building](../docs/content/building.md)), then optional example dependencies:

```bash
pip install ".[examples]"
```

Or manually:

```bash
pip install -r examples/requirements.txt
```

### Licenses and hardware

- **vismatch** is [BSD-3-Clause](https://github.com/gmberton/vismatch/blob/main/LICENSE); individual wrapped matchers may use **other licenses** — check [vismatch model details](https://github.com/gmberton/vismatch) before redistributing weights or outputs.
- **Torch + matchers:** a CUDA GPU is recommended; CPU is often supported but slow.
- **Rerun:** desktop viewer (`rerun-sdk`); see [Rerun documentation](https://www.rerun.io/docs).

## Documentation

The MkDocs manual includes an [Examples overview](../docs/content/examples_showcase.md) and a full page on [vismatch + SfM](../docs/content/examples_vismatch_sfm.md) (generic runs, SFM eval datasets, Rerun).
