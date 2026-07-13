// Nanobind spike module — minimal MVS binding to validate build toolchain.
// See docs/plans/2026-07-11-nanobind-migration.md

#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

#include "theia/mvs/view_selection_mvsnet.h"

namespace nb = nanobind;

NB_MODULE(pytheia_spike, m) {
  nb::module_ mvs = m.def_submodule("mvs");
  mvs.def("ViewSelectionMVSNet", &theia::ViewSelectionMVSNet);
}
