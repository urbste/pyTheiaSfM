#include "theia/io/io_wrapper.h"

#include "theia/io/import_nvm_file.h"
#include "theia/io/populate_image_sizes.h"
#include "theia/io/read_1dsfm.h"
#include "theia/io/read_bundler_files.h"
#include "theia/io/read_strecha_dataset.h"
#include "theia/io/reconstruction_reader.h"

namespace theia {

std::tuple<bool, Reconstruction> ImportNVMFileWrapper(
    const std::string& nvm_filepath) {
  Reconstruction reconstr = Reconstruction();
  const bool success = ImportNVMFile(nvm_filepath, &reconstr);
  return std::make_tuple(success, reconstr);
}

std::tuple<bool, Reconstruction> PopulateImageSizesAndPrincipalPointsWrapper(
    const std::string& image_directory) {
  Reconstruction reconstr = Reconstruction();
  const bool success =
      PopulateImageSizesAndPrincipalPoints(image_directory, &reconstr);
  return std::make_tuple(success, reconstr);
}

std::tuple<bool, Reconstruction, ViewGraph> Read1DSFMWrapper(
    const std::string& dataset_directory) {
  ViewGraph view_graph = ViewGraph();
  Reconstruction reconstr = Reconstruction();
  const bool success = Read1DSFM(dataset_directory, &reconstr, &view_graph);
  return std::make_tuple(success, reconstr, view_graph);
}

std::tuple<bool, Reconstruction> ReadBundlerFilesWrapper(
    const std::string& lists_file, const std::string& bundle_file) {
  Reconstruction reconstr = Reconstruction();
  const bool success = ReadBundlerFiles(lists_file, bundle_file, &reconstr);
  return std::make_tuple(success, reconstr);
}

std::tuple<bool, Reconstruction> ReadStrechaDatasetWrapper(
    const std::string& dataset_directory) {
  Reconstruction reconstr = Reconstruction();
  const bool success = ReadStrechaDataset(dataset_directory, &reconstr);
  return std::make_tuple(success, reconstr);
}

std::tuple<bool, Reconstruction> ReadReconstructionWrapper(
    const std::string& input_file) {
  Reconstruction reconstr = Reconstruction();
  const bool success = ReadReconstruction(input_file, &reconstr);
  return std::make_tuple(success, reconstr);
}

}  // namespace theia
