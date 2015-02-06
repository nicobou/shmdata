/*
 * This file is part of switcher-jack.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_AUDIO_RING_BUFFER_H__
#define __SWITCHER_AUDIO_RING_BUFFER_H__

#include <atomic>

namespace switcher {
template<typename SampleType> class AudioRingBuffer {
 public:
  explicit AudioRingBuffer(size_t size_in_sample);
  AudioRingBuffer() = delete;
  ~AudioRingBuffer() = default;
  
  // return the number of sample actually emplaced.
  size_t emplace(size_t num,
                 std::function<SampleType(*)(const size_t &pos)> sample_factory);
  
 private:
  std::vector<SampleType> buffer_;
  const size_t buffer_size_; 
  std::atomic_ulong available_size_;
};

}  // namespace switcher
#include "./audio-ring-buffer_spec.hpp"
#endif
