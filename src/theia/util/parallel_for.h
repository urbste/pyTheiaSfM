#ifndef THEIA_UTIL_PARALLEL_FOR_H_
#define THEIA_UTIL_PARALLEL_FOR_H_

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <thread>
#include <vector>

namespace theia {

// Runs fn(k) for every k in [0, n) on up to num_threads threads (the calling
// thread included). Each index is handled by exactly one thread, so results
// written per index do not depend on the thread count or scheduling.
//
// pyTheia is not built with OpenMP, so `#pragma omp parallel for` is a no-op;
// use this instead.
template <typename Fn>
void ParallelFor(const int num_threads, const size_t n, const Fn& fn) {
  const size_t threads =
      std::min(static_cast<size_t>(std::max(1, num_threads)), n);
  if (threads <= 1) {
    for (size_t k = 0; k < n; ++k) {
      fn(k);
    }
    return;
  }
  std::atomic<size_t> next(0);
  auto worker = [&]() {
    for (size_t k = next.fetch_add(1); k < n; k = next.fetch_add(1)) {
      fn(k);
    }
  };
  std::vector<std::thread> pool;
  pool.reserve(threads - 1);
  for (size_t t = 1; t < threads; ++t) {
    pool.emplace_back(worker);
  }
  worker();
  for (std::thread& thread : pool) {
    thread.join();
  }
}

}  // namespace theia

#endif  // THEIA_UTIL_PARALLEL_FOR_H_
