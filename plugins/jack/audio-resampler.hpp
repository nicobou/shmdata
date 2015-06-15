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

#ifndef __SWITCHER_AUDIO_RESAMPLER_H__
#define __SWITCHER_AUDIO_RESAMPLER_H__

#include "./audio-ring-buffer.hpp"

namespace switcher {
template<typename SampleT>
class AudioResampler {
 public:
  AudioResampler() = delete;
  explicit AudioResampler(std::size_t original_size,
                          std::size_t resampled_size,
                          const SampleT *samplebuf,
                          unsigned int channel_number = 0,
                          unsigned int number_of_channels = 1);
  inline SampleT zero_pole_get_next_sample();
  inline SampleT linear_get_next_sample();
  inline SampleT copy_get_next_sample();
 private:
  std::size_t original_size_;
  std::size_t resampled_size_;
  double ratio_;
  unsigned int channel_number_;
  unsigned int number_of_channels_;
  const SampleT *samplebuf_;
  std::size_t cur_pos_{0};
};

}  // namespace switcher
#include "./audio-resampler_spec.hpp"
#endif
