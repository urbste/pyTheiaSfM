// theia_hybrid_sfm: hybrid SfM from reconstruction.recon + view_graph.bin.
// See sfm_app_common.h for the input and output layout and theia_apps::RunSfmApp
// for the flags (--input, --out, --seed, --config, --set, --list_options).

#include "apps/sfm_app_common.h"

int main(int argc, char** argv) {
  return theia_apps::RunSfmApp(argc, argv,
                               theia::ReconstructionEstimatorType::HYBRID);
}
