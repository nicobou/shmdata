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

#include <cmath>

namespace switcher {

template <typename SampleT>
AudioResampler<SampleT>::AudioResampler(std::size_t original_size,
                                        std::size_t resampled_size,
                                        const SampleT* samplebuf,
                                        unsigned int channel_number,
                                        unsigned int number_of_channels)
    : original_size_(original_size),
      resampled_size_(resampled_size),
      ratio_((double)(original_size - 1) / (double)(resampled_size - 1)),
      channel_number_(channel_number),
      number_of_channels_(number_of_channels),
      samplebuf_(samplebuf) {}

template <typename SampleT>
SampleT AudioResampler<SampleT>::zero_pole_get_next_sample() {
  if (cur_pos_ >= resampled_size_) {
    cur_pos_ = resampled_size_ - 1;  // cliping
#ifdef DEBUG
    std::cerr << __FUNCTION__ << " is asked for more sample than supposed";
#endif
  }
  std::size_t new_pos = (std::size_t)(ratio_ * (double)cur_pos_);
  if (cur_pos_ < resampled_size_ - 1) ++cur_pos_;
  return samplebuf_[new_pos * number_of_channels_ + channel_number_];
}

template <typename SampleT>
SampleT AudioResampler<SampleT>::linear_get_next_sample() {
  double pos = (double)cur_pos_ * ratio_;
  double decimals = pos - std::floor(pos);
  if (cur_pos_ >= resampled_size_) {
    cur_pos_ = resampled_size_ - 1;  // cliping
#ifdef DEBUG
    std::cerr << __FUNCTION__ << " is asked for more sample than supposed";
#endif
  }
  SampleT sample =
      (1. - decimals) *
          samplebuf_[(std::size_t)std::floor(pos) * number_of_channels_ + channel_number_] +
      decimals * samplebuf_[(std::size_t)std::ceil(pos) * number_of_channels_ + channel_number_];
  if (cur_pos_ < resampled_size_ - 1) ++cur_pos_;
  return sample;
}

}  // namespace switcher
