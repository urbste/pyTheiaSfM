# Contributing to Theia {#chapter-contributing}

We welcome and encourage contributions to Theia, whether they are new features, bug fixes or tests. The [Theia mailing list](http://groups.google.com/group/theia-vision-library) is the best place for all development related discussions. Please consider joining it. If you have an idea for how you'd like to contribute to Theia, please consider emailing the list first to voice your idea. We can help you fine-tune your idea and this will also help avoid duplicate work by somebody else who may be working on the same feature.

## Style and Testing

We follow Google's [C++ Style Guide](https://google.github.io/styleguide/cppguide.html) and use git for version control. Contributions are reviewed via GitHub pull requests.

When contributing substantial new code or bug fixes, please add unit tests to ensure the usage of the code (or to prove the bug is fixed!).

## Dual-language examples in the manual (Python / C++)

The site uses [MkDocs Material](https://squidfunk.github.io/mkdocs-material/) **content tabs** so readers can switch between equivalent Python and C++ snippets. Install the doc dependencies from the repository root:

```bash
pip install -r docs/requirements.txt
# or: pip install ".[docs]"
```

Use tab titles and a **four-space indent** for the body under each tab (same pattern as [Material’s content tabs](https://squidfunk.github.io/mkdocs-material/reference/content-tabs/)):

=== "Python"

    ```python
    import pytheia as pt
    rec = pt.sfm.Reconstruction()
    ```

=== "C++"

    ```cpp
    #include "theia/sfm/reconstruction.h"

    theia::Reconstruction reconstruction;
    ```

The theme is configured with `content.tabs.link`, so when several tab groups on a page use the same labels (e.g. **Python** / **C++**), the selected tab stays in sync while navigating that page.

## CMake

We use CMake to generate makefiles for Theia to maximize the cross-platform usability. If you need to add a new library or a new file to Theia, you will likely need to add that file to the CMakeLists.txt in src/theia (along with a unit test!).

## Developing for Theia

Much of the instructions that follow in this section were borrowed and modified from the [Ceres Solver](http://homes.cs.washington.edu/~sagarwal/ceres-solver/stable/contributing.html) project.

1.  Download and configure `git`.

    -   Mac `brew install git`.
    -   Linux `sudo apt-get install git`.
    -   Windows. Download [msysgit](https://code.google.com/p/msysgit/), which includes a minimal [Cygwin](http://www.cygwin.com/) install.

2.  Sign up for a [GitHub](http://github.com) account. Accounts are free to register.

3.  Clone the pyTheia repository:

    ``` bash
    git clone https://github.com/urbste/pyTheiaSfM.git
    ```

4.  Build Theia, following the instructions in [chapter-building](building.md#chapter-building).

5.  Open a pull request on [GitHub](https://github.com/urbste/pyTheiaSfM) against `master`.

## Submitting a change

1.  Fork the repository and create a feature branch from `master`.
2.  Make your changes; include tests when adding behavior or fixing bugs.
3.  Regenerate or update `.pyi` stubs if you change Python bindings (`dev/generate_stubs.sh` or a full wheel build).
4.  Open a pull request with a clear description and test notes.
5.  Address review feedback; rebase on `master` if needed before merge.
