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
#include <typeinfo>

#include "../logger/logger.hpp"

namespace switcher {
namespace quiddities {

template <typename SampleT>
AudioResampler<SampleT>::AudioResampler(std::shared_ptr<spdlog::logger> logger,
                                        unsigned int number_of_channels)
    : logger_(logger),
      number_of_channels_(number_of_channels),
      resampler_config_(src_new(SRC_SINC_FASTEST, number_of_channels_, &error_)) {
  if (nullptr == resampler_config_) {
    LOGGER_WARN(logger_,
                "resample quiddity could not intialize the resample: %",
                std::string(src_strerror(error_)));
    return;
  };
  resampler_data_ = std::make_unique<SRC_DATA>();
}

template <typename SampleT>
void AudioResampler<SampleT>::do_resample(std::size_t original_size,
                                          std::size_t resampled_size,
                                          const SampleT* samplebuf) {
  resampler_data_->end_of_input = 0;
  resampler_data_->src_ratio =
      static_cast<double>(resampled_size) / static_cast<double>(original_size);
  // configure resampler
  resampler_data_->input_frames = original_size;
  resampler_data_->output_frames = resampler_data_->input_frames * resampler_data_->src_ratio;
  // size buffer
  resampled_.resize(resampler_data_->output_frames * number_of_channels_ * sizeof(float));
  // configure resampler memory
  set_resampler_data(const_cast<SampleT*>(samplebuf), original_size);
  // do resampling
  auto error = src_process(resampler_config_, resampler_data_.get());
  if (error) {
    LOGGER_ERROR(logger_, "resample error: {}", std::string(src_strerror(error)));
    return;
  }
}

template <typename SampleT>
float AudioResampler<SampleT>::get_sample(std::size_t pos, unsigned int channel_number) {
  return resampled_[pos * number_of_channels_ + channel_number];
}

}  // namespace quiddities
}  // namespace switcher
