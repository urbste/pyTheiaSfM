# Python API overview {#chapter-python-wrapper}

pyTheia is a normal Python package with a compiled extension. After [building](building.md), import the public namespace as usual:

``` python
import pytheia as pt
```

The top-level package re-exports everything from the native submodule `pytheia.pytheia`. You normally **do not** import `pytheia.pytheia` in application code; use `import pytheia` (or `import pytheia as pt`) instead.

Submodules correspond to the same areas as in the C++ library (see the [API Reference](api.md)). They are exposed as attributes of `pt`:

| Submodule | Role (short) |
|-----------|----------------|
| `pt.io` | Reading and writing reconstructions and related data. |
| `pt.math` | Math helpers and Sophus Lie-group bindings. |
| `pt.matching` | Feature matching and correspondence utilities. |
| `pt.mvs` | Multi-view stereo oriented helpers. |
| `pt.sfm` | Reconstruction, BA, cameras, tracks, pipelines. |
| `pt.solvers` | Geometric estimation and solvers. |

## Type information (PEP 561)

The wheel ships `py.typed` and `.pyi` stubs next to the extension so editors and mypy/pyright can resolve symbols without loading the binary.

-   Stubs are generated with `pybind11-stubgen` during development (see `pip install ".[dev]"` in the root `pyproject.toml`).

-   Regenerate after changing bindings:

    ``` bash
    pip install ".[dev]"
    export PYTHONPATH=$(pwd)/src
    python -m pybind11_stubgen pytheia.pytheia -o src
    ```

    The project build (`python setup.py bdist_wheel`) runs the same step when `pybind11-stubgen` is installed and `GENERATE_STUBS` is not `0`.

## Examples in the repo

- **`pyexamples/`** — small, self-contained scripts (OpenCV features, classic matchers, NeRFStudio export helpers). Fewer optional dependencies.
- **`examples/`** — showcase workflows: **vismatch** matchers, **Rerun** visualization, and **Gaussian splatting** export notes. Install optional deps with `pip install ".[examples]"` (see repository `examples/requirements.txt`).
- **`pytests/`** — unit and integration tests.

See [Examples showcase](examples_showcase.md) for a map of both trees.

## Python versus C++ (tabbed snippets)

The HTML manual can show the same workflow in both languages. Use the pattern from [chapter-contributing](contributions.md#chapter-contributing) (dual-language examples). For instance:

=== "Python"

    ``` python
    import pytheia as pt
    
    rec = pt.sfm.Reconstruction()
    ```

=== "C++"

    ``` cpp
    #include <theia/sfm/reconstruction.h>
    
    theia::sfm::Reconstruction reconstruction;
    ```

## Relationship to the C++ reference

Chapters under [api](api.md) are written in the C++ domain and describe the underlying Theia types. Python names and signatures follow the bindings in `src/pytheia/` (pybind11). Where names match, the C++ chapter is the conceptual reference; use the stubs or `help()` on the Python objects for the exact wrapper surface.
