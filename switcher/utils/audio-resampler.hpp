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
namespace utils {

/**
 * An audio resampling class. Once initialized, the resampler can be feed with new buffers
 * continuously. However, once a new buffer has been requested for resampling, then resampled buffer
 * from previous one is replaced.
 *
 * Internaly, the class convert samples into float if necessary.
 * \tparam Audio sample type.
 */
template <typename SampleT>
class AudioResampler {
 public:
  AudioResampler() = delete;
  /**
   * Construct an AudioResampler object
   * \param logger              Logger where to write logs.
   * \param number_of_channels  Number of channels of audio buffers.
   **/
  explicit AudioResampler(std::shared_ptr<spdlog::logger> logger,
                          unsigned int number_of_channels = 1);
  /**
   * Resample a new audio buffer.
   * \param original_size Number of samples, for a single channel, of the buffer to resample.
   * \param resampled_size Expected number of sample after resampling.
   * \param samplebuf Buffer containing the samples to resample.
   **/
  void do_resample(std::size_t original_size, std::size_t resampled_size, const SampleT* samplebuf);
  /**
   * Get a single sample in the float format.
   * \param pos Sample position in the channel.
   * \param channel_number Channel number for this sample (starting at 1).
   * \return A sample.
   */
  inline float get_sample(std::size_t pos, unsigned int channel_number);

 private:
  /**
   * Template overload in order to configure the resampler data with the float format.
   */
  template <typename T = SampleT>
  std::enable_if_t<std::is_same<T, float>::value> set_resampler_data(
      T* samplebuf, std::size_t /*original_size*/) {
    resampler_data_->data_in = static_cast<float*>(samplebuf);
    resampler_data_->data_out = resampled_.data();
  }

  /**
   * Template overload in order to configure the resampler data with the short format.
   */
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

  /**
   * Template overload in order to configure the resampler data with the int format.
   */
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
  std::shared_ptr<spdlog::logger> logger_;      //!< Logger where to write logs.
  unsigned int number_of_channels_;             //!< Number of channels of the audio stream.
  SRC_STATE* resampler_config_{nullptr};        //!< Configuration for libsamplerate.
  std::unique_ptr<SRC_DATA> resampler_data_{};  //!< Data to provide to libsamplerate.
  std::vector<float> resampled_{};              //!< Buffer of resampled data.
  std::size_t in_converted_size_{0};            //!< Size of in_converted.
  std::vector<float> in_converted_{};           //!< Float-converted input buffer.
  int error_{0};                                //!< libsamplerate error handling.
};

}  // namespace utils
}  // namespace switcher
#include "./audio-resampler_spec.hpp"
#endif
