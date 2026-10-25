// Copyright (C) 2013 The Regents of the University of California (Regents).
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

#include "theia/util/random.h"

#include <atomic>
#include <chrono>  // NOLINT
#include <cstdlib>
#include <glog/logging.h>
#include <random>
#include <string>

#include "theia/util/util.h"

namespace theia {
namespace {

#ifdef THEIA_HAS_THREAD_LOCAL_KEYWORD
thread_local std::mt19937 util_generator;
#else
static std::mt19937 util_generator;
#endif  // THEIA_HAS_THREAD_LOCAL_KEYWORD

// Process-wide override for the seed used by the default constructor. Every
// RANSAC sampler and global estimator that is not handed an RNG constructs one
// with the default constructor, which reseeds this thread's engine. With an
// override set, each of them starts from the same state no matter which thread
// runs it, so results no longer depend on the clock or on thread scheduling.
// Initialised from THEIA_RANDOM_SEED (decimal) if set; -1 means "use the clock".
std::atomic<long long> default_seed_override{[] {
  const char* env = std::getenv("THEIA_RANDOM_SEED");
  if (env == nullptr || *env == '\0') return -1LL;
  try {
    return static_cast<long long>(std::stoul(env));
  } catch (...) {
    return -1LL;
  }
}()};

}  // namespace

void SetDefaultRandomSeed(const unsigned seed) {
  default_seed_override.store(static_cast<long long>(seed));
}

void ClearDefaultRandomSeed() { default_seed_override.store(-1LL); }

bool HasDefaultRandomSeed() { return default_seed_override.load() >= 0; }

RandomNumberGenerator::RandomNumberGenerator() {
  const long long override_seed = default_seed_override.load();
  const unsigned seed =
      override_seed >= 0
          ? static_cast<unsigned>(override_seed)
          : static_cast<unsigned>(
                std::chrono::system_clock::now().time_since_epoch().count());
  util_generator.seed(seed);
}

RandomNumberGenerator::RandomNumberGenerator(const unsigned seed) {
  util_generator.seed(seed);
}

void RandomNumberGenerator::Seed(const unsigned seed) {
  util_generator.seed(seed);
}

// Get a random double between lower and upper (inclusive).
double RandomNumberGenerator::RandDouble(const double lower,
                                         const double upper) {
  std::uniform_real_distribution<double> distribution(lower, upper);
  return distribution(util_generator);
}

float RandomNumberGenerator::RandFloat(const float lower, const float upper) {
  std::uniform_real_distribution<float> distribution(lower, upper);
  return distribution(util_generator);
}

// Get a random int between lower and upper (inclusive).
int RandomNumberGenerator::RandInt(const int lower, const int upper) {
  std::uniform_int_distribution<int> distribution(lower, upper);
  return distribution(util_generator);
}

// Gaussian Distribution with the corresponding mean and std dev.
double RandomNumberGenerator::RandGaussian(const double mean,
                                           const double std_dev) {
  std::normal_distribution<double> distribution(mean, std_dev);
  return distribution(util_generator);
}

Eigen::Vector2d RandomNumberGenerator::RandVector2d(const double min,
                                                    const double max) {
  return Eigen::Vector2d(RandDouble(min, max), RandDouble(min, max));
}

Eigen::Vector2d RandomNumberGenerator::RandVector2d() {
  return RandVector2d(-1.0, 1.0);
}

Eigen::Vector3d RandomNumberGenerator::RandVector3d(const double min,
                                                    const double max) {
  return Eigen::Vector3d(
      RandDouble(min, max), RandDouble(min, max), RandDouble(min, max));
}

Eigen::Vector3d RandomNumberGenerator::RandVector3d() {
  return RandVector3d(-1.0, 1.0);
}

Eigen::Vector4d RandomNumberGenerator::RandVector4d(const double min,
                                                    const double max) {
  return Eigen::Vector4d(RandDouble(min, max),
                         RandDouble(min, max),
                         RandDouble(min, max),
                         RandDouble(min, max));
}

Eigen::Vector4d RandomNumberGenerator::RandVector4d() {
  return RandVector4d(-1.0, 1.0);
}

}  // namespace theia
