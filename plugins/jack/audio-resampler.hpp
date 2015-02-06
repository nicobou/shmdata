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
  using resample_fun_t = std::function<SampleT(*)(const size_t &pos)>; 
  template<typename SampleT>
  bool resample_in_ring_buffer(const size_t num_channels,
                               resample_fun_t<SampleT> fun);
  
  template<typename SampleT>
  SampleT zero_pole_get_sample(const size_t &pos);
 private:
  size_t original_size_;
  size_t resampled_size_;
  SampleT samplebuf_[];
};

}  // namespace switcher
#include "./audio-resampler_spec.hpp"
#endif
