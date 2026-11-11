// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_PARALLEL_FOR_H_
#define THEIA_SFM_INIT_FREE_PARALLEL_FOR_H_

#include <algorithm>
#include <thread>
#include <vector>

namespace theia {

// Number of contiguous chunks ParallelFor splits [0, num_items) into: at most
// num_threads, and at least min_items_per_chunk items per chunk (so that small
// problems run on the calling thread without spawning threads).
inline int NumParallelChunks(const int num_threads,
                             const int num_items,
                             const int min_items_per_chunk) {
  if (num_items <= 0) {
    return 0;
  }
  const int by_size =
      std::max(1, num_items / std::max(1, min_items_per_chunk));
  return std::max(1, std::min(num_threads, by_size));
}

// Calls fn(begin, end, chunk) for the NumParallelChunks(...) contiguous chunks
// of [0, num_items), one thread per chunk; chunk 0 runs on the calling thread.
// fn must only write to data owned by its chunk (or to per-chunk buffers).
template <typename Function>
void ParallelFor(const int num_threads,
                 const int num_items,
                 const int min_items_per_chunk,
                 const Function& fn) {
  const int num_chunks =
      NumParallelChunks(num_threads, num_items, min_items_per_chunk);
  if (num_chunks == 0) {
    return;
  }
  if (num_chunks == 1) {
    fn(0, num_items, 0);
    return;
  }
  auto chunk_begin = [&](const int chunk) {
    return static_cast<int>(static_cast<long long>(num_items) * chunk /
                            num_chunks);
  };
  std::vector<std::thread> threads;
  threads.reserve(num_chunks - 1);
  for (int chunk = 1; chunk < num_chunks; ++chunk) {
    threads.emplace_back(
        [&, chunk]() { fn(chunk_begin(chunk), chunk_begin(chunk + 1), chunk); });
  }
  fn(0, chunk_begin(1), 0);
  for (std::thread& thread : threads) {
    thread.join();
  }
}

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_PARALLEL_FOR_H_
