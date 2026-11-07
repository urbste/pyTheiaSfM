// Parsing of ReconstructionEstimatorOptions for the native command line
// applications. Options can come from a JSON file (nested objects are option
// groups) and from repeated "dotted.key=value" overrides. The semantics match
// the Python CLI (`pytheia sfm --config/--set`):
//   * Defaults first, then the JSON file, then the --set overrides.
//   * Enums are given by name (case-insensitive) or by integer value.
//   * Bit masks (OptimizeIntrinsicsType) accept "A|B|C" names or an integer.
//   * Booleans accept true/false, yes/no, on/off, 1/0.
//   * Unknown keys are errors.
//
// The shared_ptr RandomNumberGenerator fields are not settable here; use
// --seed instead.

#ifndef THEIA_APPS_OPTIONS_JSON_H_
#define THEIA_APPS_OPTIONS_JSON_H_

#include <string>
#include <vector>

#include "theia/sfm/reconstruction_estimator_options.h"

namespace theia_apps {

// Applies every key of a JSON object file to `options`. Returns false and
// fills `error` on a read, parse, or option error.
bool ApplyOptionsJsonFile(const std::string& path,
                          theia::ReconstructionEstimatorOptions* options,
                          std::string* error);

// Applies one override of the form "key=value" (key may be dotted, e.g.
// "ligt_position_estimator_options.use_scale_constraint=false").
bool ApplyOptionSetting(const std::string& setting,
                        theia::ReconstructionEstimatorOptions* options,
                        std::string* error);

// Formats a double like Python's repr(float): the shortest round-trip digits,
// always with a decimal point ("10.0", "0.9999", "1e-08").
std::string FormatDouble(double value);

// Every settable leaf option with its current value, as "name = value" lines.
std::vector<std::string> ListOptions(
    const theia::ReconstructionEstimatorOptions& options);

// Removes every "--name VALUE" and "--name=VALUE" pair from argv (gflags only
// keeps the last value of a repeated flag). Returns the remaining argv,
// including argv[0], as pointers into the original argv. Values are appended
// to `values` in order of appearance. Parsing stops at a lone "--".
std::vector<char*> ExtractRepeatedFlag(const std::string& name,
                                       int argc,
                                       char** argv,
                                       std::vector<std::string>* values);

}  // namespace theia_apps

#endif  // THEIA_APPS_OPTIONS_JSON_H_
