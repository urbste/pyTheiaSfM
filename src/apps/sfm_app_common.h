// Shared driver for theia_global_sfm, theia_incremental_sfm and theia_hybrid_sfm.
//
// Inputs (same layout as `pytheia sfm --input DIR`):
//   DIR/reconstruction.recon  views and tracks with features (ReadReconstruction)
//   DIR/view_graph.bin        view graph (ViewGraph::ReadFromDisk)
//
// Outputs (same as `pytheia sfm --out DIR`):
//   DIR/reconstruction.recon, DIR/reconstruction.ply, DIR/summary.json

#ifndef THEIA_APPS_SFM_APP_COMMON_H_
#define THEIA_APPS_SFM_APP_COMMON_H_

#include "theia/sfm/reconstruction_estimator_options.h"

namespace theia_apps {

// Parses the command line and runs the estimator selected by `method`.
// Returns the process exit status: 0 on success, 1 on a runtime failure,
// 2 on a usage error.
int RunSfmApp(int argc, char** argv, theia::ReconstructionEstimatorType method);

}  // namespace theia_apps

#endif  // THEIA_APPS_SFM_APP_COMMON_H_
