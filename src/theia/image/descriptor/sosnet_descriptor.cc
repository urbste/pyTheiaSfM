// Copyright (C) 2016 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/image/descriptor/sosnet_descriptor.h"

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "akaze/src/AKAZE.h"
#include "glog/logging.h"

#include "theia/image/image.h"
#include "theia/image/keypoint_detector/keypoint.h"

#include <torch/script.h>

namespace theia {

// Read model
std::shared_ptr<torch::jit::script::Module> ReadPyTorchModel(
        const std::string& model_path, const bool use_gpu) {
  std::shared_ptr<torch::jit::script::Module> model;
  if (use_gpu) {
     model = torch::jit::load(model_path,torch::kCUDA);
  }
  else {
     model = torch::jit::load(model_path);
  }
  assert(model != nullptr);
  return model;
}

torch::Tensor ConvertImageToTensor(const cv::Mat& image) {

  const int n_channels = image.channels();
  const int image_type = image.type();

  // Image Type must be one of CV_8U, CV_32F, CV_64F
  assert((image_type % 8 == 0) || ((image_type - 5) % 8 == 0) || ((image_type - 6) % 8 == 0));

  std::vector<int64_t> dims = {1, SOSNET_PATCH_SIZE, SOSNET_PATCH_SIZE, n_channels};
  std::vector<int64_t> permute_dims = {0, 3, 1, 2};

  torch::Tensor image_as_tensor;
  //cv::Mat image = images[i].clone();

  if (image_type % 8 == 0) {
    torch::TensorOptions options(torch::kUInt8);
    image_as_tensor = torch::from_blob(image.data, torch::IntList(dims), options).clone();
  } else if ((image_type - 5) % 8 == 0) {
    torch::TensorOptions options(torch::kFloat32);
    image_as_tensor = torch::from_blob(image.data, torch::IntList(dims), options).clone();
  } else if ((image_type - 6) % 8 == 0) {
    torch::TensorOptions options(torch::kFloat64);
    image_as_tensor = torch::from_blob(image.data, torch::IntList(dims), options).clone();
  }

  image_as_tensor = image_as_tensor.permute(torch::IntList(permute_dims));
  image_as_tensor = image_as_tensor.toType(torch::kFloat32);

  //images_as_tensors.push_back(image_as_tensor);
  //torch::Tensor output_tensor = torch::cat(images_as_tensors, 0);

  return image_as_tensor;
}

void DescribeOpenCV(const cv::Mat& image,
                    const std::vector<libAKAZE::AKAZEKeypoint>& keypoints,
                    const std::shared_ptr<torch::jit::script::Module> model,
                    const float mag_factor,
                    const bool use_gpu,
                    const bool upright,
                    cv::Mat& descriptors) {
  cv::Size patch_size = cv::Size(SOSNET_PATCH_SIZE, SOSNET_PATCH_SIZE);
  std::vector<cv::Mat> patches(keypoints.size());
  std::vector<torch::Tensor> patches_as_tensors(keypoints.size());

  for (size_t i = 0; i < keypoints.size(); ++i) {
    const float s = static_cast<float>(mag_factor * keypoints[i].size) / SOSNET_PATCH_SIZE;
    const float angle = keypoints[i].angle * SOSNET_RHO;
    const float cos = std::cos(angle);
    const float sin = std::sin(angle);

    cv::Mat A23 = cv::Mat(2,3, CV_32FC1);
    A23.at<float>(0,0) = s * cos;
    A23.at<float>(1,0) = s * sin;
    A23.at<float>(0,1) =-s * sin;
    A23.at<float>(1,1) = s * cos;
    A23.at<float>(0,2) =(-s*cos + s * sin) * SOSNET_PATCH_SIZE_2 + keypoints[i].pt(0);
    A23.at<float>(1,2) =(-s*sin - s * cos) * SOSNET_PATCH_SIZE_2 + keypoints[i].pt(1);
    cv::Mat patch;
    cv::warpAffine(image, patch, A23, patch_size,
                   cv::WARP_INVERSE_MAP + cv::INTER_CUBIC + cv::WARP_FILL_OUTLIERS);
    patches_as_tensors[i] = ConvertImageToTensor(patch);
  }
  torch::Tensor input_tensor = torch::cat(patches_as_tensors, 0);

  std::vector<torch::jit::IValue> inputs;
  if (use_gpu) {
    input_tensor = input_tensor.to(at::kCUDA);
  }
  inputs.push_back(input_tensor);

  // Execute the model and turn its output into a tensor.
  torch::Tensor output = model->forward(inputs).toTensor();
  if (use_gpu) {
    output = output.to(torch::kCPU);
  }
  cv::Mat descriptors_temp = cv::Mat(cv::Size(128,keypoints.size()), CV_32FC1, output.data_ptr());
  descriptors = descriptors_temp.clone();
}

SOSNetDescriptorExtractor::SOSNetDescriptorExtractor(const SOSNetParameters &detector_params) :
    sosnet_params_(detector_params) {
   LOG(INFO) << "Loading SOSNet model";
   sosnet = ReadPyTorchModel(sosnet_params_.model_path,sosnet_params_.use_gpu);
   LOG(INFO) << "Finished loading SOSNet model";
}


bool SOSNetDescriptorExtractor::ComputeDescriptor(const FloatImage& image,
                                                 const Keypoint& keypoint,
                                                 Eigen::VectorXf* descriptor) {
  LOG(FATAL) << "SOSNet must use its own Keypoints and so calling the AKAZE "
                "descriptor extractor with different keypoint cannot be used. "
                "Please use "
                "SOSNetDescriptorExtractor::DetectAndExtractDescriptors() "
                "instead.";
}

bool SOSNetDescriptorExtractor::DetectAndExtractDescriptors(
    const FloatImage& image,
    std::vector<Keypoint>* keypoints,
    std::vector<Eigen::VectorXf>* descriptors) {
  // Try to convert the image to grayscale and eigen type.
  const FloatImage& gray_image = image.AsGrayscaleImage();
  libAKAZE::RowMatrixXf img_32 =
      Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                                     Eigen::RowMajor> >(
          gray_image.Data(), gray_image.Rows(), gray_image.Cols());

  // Set the akaze options.
  libAKAZE::AKAZEOptions options;
  options.img_width = img_32.cols();
  options.img_height = img_32.rows();
  options.num_threads = 1;
  options.soffset = 1.6f;
  options.derivative_factor = 1.5f;
  options.omax = sosnet_params_.maximum_octave_levels;
  options.nsublevels = sosnet_params_.num_sublevels;
  options.dthreshold = sosnet_params_.hessian_threshold;
  options.min_dthreshold = 0.00001f;

  options.diffusivity = libAKAZE::PM_G2;
  options.descriptor = libAKAZE::MSURF;
  options.descriptor_size = 0;
  options.descriptor_channels = 3;
  options.descriptor_pattern_size = 10;
  options.sderivatives = 1.0;

  options.kcontrast = 0.001f;
  options.kcontrast_percentile = 0.7f;
  options.kcontrast_nbins = 300;

  options.verbosity = false;

  // Extract features.
  std::vector<libAKAZE::AKAZEKeypoint> akaze_keypoints;
  libAKAZE::AKAZE evolution(options);
  evolution.Create_Nonlinear_Scale_Space(img_32);
  evolution.Feature_Detection(akaze_keypoints);

  cv::Mat image_cv;
  cv::eigen2cv(img_32, image_cv);

  // Compute descriptors using SOSNet
  //libAKAZE::AKAZEDescriptors akaze_descriptors;
  //evolution.Compute_Descriptors(akaze_keypoints, akaze_descriptors);
  cv::Mat deep_descriptors;
  DescribeOpenCV(image_cv, akaze_keypoints, sosnet, 3.0f, sosnet_params_.use_gpu, false, deep_descriptors);

  // Set the output keypoints.
  keypoints->reserve(akaze_keypoints.size());
  for (const auto& akaze_keypoint : akaze_keypoints) {
    Keypoint keypoint(akaze_keypoint.pt.x(), akaze_keypoint.pt.y(),
                      Keypoint::AKAZE);
    keypoint.set_scale(akaze_keypoint.size);
    keypoint.set_strength(akaze_keypoint.response);
    keypoint.set_orientation(akaze_keypoint.angle);
    keypoints->emplace_back(keypoint);
  }

  // Set the output descriptors.
  descriptors->resize(akaze_keypoints.size());
  for (int i=0; i < akaze_keypoints.size(); ++i) {
      Eigen::Matrix<float, Eigen::Dynamic, 1> d;
      cv::cv2eigen(deep_descriptors.row(i).t(),d);
      (*descriptors)[i] = d;
  }
  return true;
}

}  // namespace theia
