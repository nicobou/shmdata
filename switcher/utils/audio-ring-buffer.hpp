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
#include <functional>
#include <vector>

namespace switcher {
namespace utils {

/**
 * A ring buffer class for multithreaded exchange of audio samples. The sample format is set using a
 * class template argument.
 *
 * The ring buffer is designed to host a single channel audio. Multichannel should be implemented
 * using multiple AudioRingBuffer.
 * \tparam SampleType Audio sample type.
 */
template <typename SampleType>
class AudioRingBuffer {
 public:
  /**
   * Construct an AudioRingBuffer object.
   * \param size_in_sample Total lenght of the ring buffer, in sample.
   */
  AudioRingBuffer(std::size_t size_in_sample = 96000);
  ~AudioRingBuffer() = default;
  /**
   * Add samples to the ring buffer. The samples are actually added one by one, so the sample must
   * be given using a sample factory function.
   * \param   num              Number of sample to add.
   * \param   sample_factory   A function that will br called num times and returns one by one the
   *                           samples to add to the ring buffer.
   * \return Number of sample actually processed.
   **/
  std::size_t put_samples(std::size_t num, std::function<SampleType()> sample_factory);
  /**
   * Pop samples from the ring buffer to a user-provided buffer.
   * \param   num              Number of sample to pop.
   * \param   dest             Buffer where to add the sample. It must be previously allocated.
   * \return Number of sample actually processed.
   **/
  std::size_t pop_samples(std::size_t num, SampleType* dest);
  /**
   * Pop samples from the ring buffer to a user-provided buffer. Instead of writing sample at each
   *sequential positions in the destination buffer, write the sample as if samples from the ring
   *buffer are a specific channel of interleaved audio.
   * \param   num              Number of sample to pop.
   * \param   dest             Buffer where to add the sample. It must be previously allocated.
   * \param   chan             Channel position the sample should accupy in the destination
   *                           buffer. First channel is number 1.
   * \param   total_chan       Number of channels targeted by the destination buffer.
   * \return Number of sample actually processed.
   **/
  std::size_t pop_samples_as_channel(std::size_t num,
                                     SampleType* dest,
                                     unsigned int chan,
                                     unsigned int total_chans);
  /**
   * Shrink the ring buffer. Removing starts from the older sample.
   * \param size             Number of sample to remove.
   * \return Number of samples dropped.
   **/
  std::size_t shrink_to(std::size_t size);
  /**
   * Get usage.
   * \return Number of sample available in the ring buffer.
   **/
  std::size_t get_usage();

 private:
  const std::size_t buffer_size_;     //!< Initial buffer size.
  std::vector<SampleType> buffer_;    //!< Actual buffer.
  std::atomic_ulong available_size_;  //!< Number of sample that can be added.
  std::size_t read_{0};               //!< Read playhead.
  std::size_t write_{0};              //!< Write playhead.
};

}  // namespace utils
}  // namespace switcher
#include "./audio-ring-buffer_spec.hpp"
#endif
