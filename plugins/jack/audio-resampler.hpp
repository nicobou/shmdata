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


namespace switcher {
template<typename TimeType> class AudioResampler {
 public:
  AudioResampler() = default;
  ~AudioResampler() = default;
  // this is returning the duration this buffer should have
  TimeType set_current_buffer_info(TimeType date,
                                   TimeType duration);
  
 private:
  TimeType current_buffer_date_{0};
  TimeType current_buffer_duration_{0};
  double ratio_{1};
  double smoothing_factor_{0.001};
};

}  // namespace switcher
#include "./audio-resampler_spec.hpp"
#endif
