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

#include <samplerate.h>

#include "./audio-ring-buffer.hpp"

namespace switcher {
namespace quiddities {
template <typename SampleT>
class AudioResampler {
 public:
  AudioResampler() = delete;
  explicit AudioResampler(std::shared_ptr<spdlog::logger> logger,
                          unsigned int number_of_channels = 1);
  void do_resample(std::size_t original_size, std::size_t resampled_size, const SampleT* samplebuf);
  inline float get_sample(std::size_t pos, unsigned int channel_number);

 private:
  template <typename T = SampleT>
  std::enable_if_t<std::is_same<T, float>::value> set_resampler_data(
      T* samplebuf, std::size_t /*original_size*/) {
    resampler_data_->data_in = static_cast<float*>(samplebuf);
    resampler_data_->data_out = resampled_.data();
  }

  template <typename T = SampleT>
  std::enable_if_t<std::is_same<T, short>::value> set_resampler_data(T* samplebuf,
                                                                     std::size_t original_size) {
    auto required_sample_for_convertion = original_size * sizeof(float) / sizeof(SampleT);
    if (in_converted_size_ < required_sample_for_convertion) {
      in_converted_size_ = required_sample_for_convertion;
      in_converted_.resize(in_converted_size_);
    }
    src_short_to_float_array(
        static_cast<short*>(samplebuf), in_converted_.data(), original_size / sizeof(short));
    resampler_data_->data_in = in_converted_.data();
    resampler_data_->data_out = resampled_.data();
  }

  template <typename T = SampleT>
  std::enable_if_t<std::is_same<T, int>::value> set_resampler_data(T* samplebuf,
                                                                   std::size_t original_size) {
    auto required_sample_for_convertion = original_size * sizeof(float) / sizeof(SampleT);
    if (in_converted_size_ < required_sample_for_convertion) {
      in_converted_size_ = required_sample_for_convertion;
      in_converted_.resize(in_converted_size_);
    }
    src_int_to_float_array(
        static_cast<int*>(samplebuf), in_converted_.data(), original_size / sizeof(int));
    resampler_data_->data_in = in_converted_.data();
    resampler_data_->data_out = resampled_.data();
  }
  std::shared_ptr<spdlog::logger> logger_;
  unsigned int number_of_channels_;
  SRC_STATE* resampler_config_{nullptr};
  std::unique_ptr<SRC_DATA> resampler_data_{};
  std::vector<float> resampled_{};  //!< buffer of resampled data
  std::size_t in_converted_size_{0};
  std::vector<float> in_converted_{};  //!< depth converted input buffer
  int error_{0};
};

}  // namespace quiddities
}  // namespace switcher
#include "./audio-resampler_spec.hpp"
#endif
