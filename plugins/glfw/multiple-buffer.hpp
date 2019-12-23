/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef __SWITCHER_MULTIPLE_BUFFER_H__
#define __SWITCHER_MULTIPLE_BUFFER_H__

#include <array>
#include <atomic>
#include <mutex>
#include <vector>

template <int buffer_depth, typename T>
class MultipleBuffer {
 public:
  MultipleBuffer() = default;
  ~MultipleBuffer() = default;
  MultipleBuffer(const MultipleBuffer&) = delete;

  const T* read(bool& has_changed) {
    std::lock_guard<std::mutex> lock(access_m_);
    auto ret = data_[read_index_].data();
    if (read_index_ != write_index_)
      read_index_ = (read_index_ + 1) % buffer_depth;
    else
      has_changed = false;
    return ret;
  }

  void write(T* data_start, T* data_end) {
    std::lock_guard<std::mutex> lock(access_m_);
    auto next_write = (write_index_ + 1) % buffer_depth;
    if (read_index_ == next_write) next_write = (next_write + 1) % 3;
    write_index_ = next_write;
    std::copy(data_start, data_end, data_[next_write].data());
  }

  void resize(size_t size) {
    std::lock_guard<std::mutex> lock(access_m_);
    for (auto& buffer : data_) {
      buffer.resize(size);
    }
  }

 private:
  std::mutex access_m_{};
  std::atomic<int> read_index_{0};
  std::atomic<int> write_index_{0};
  std::array<std::vector<T>, buffer_depth> data_{};
};

#endif
