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
  AudioRingBuffer(std::size_t size_in_sample = 96000);
  ~AudioRingBuffer() = default;
  // put and pop are returning the number of sample actually processed
  std::size_t put_samples(std::size_t num, std::function<SampleType()> sample_factory);
  std::size_t pop_samples(std::size_t num, SampleType *dest);
  // return the number of samples dropped
  std::size_t shrink_to(std::size_t size);
  std::size_t get_usage();  
  
 private:
  const std::size_t buffer_size_; 
  std::vector<SampleType> buffer_;
  std::atomic_ulong available_size_;
  std::size_t read_{0};
  std::size_t write_{0};
};

}  // namespace switcher
#include "./audio-ring-buffer_spec.hpp"
#endif
