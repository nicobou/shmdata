/*
 * This file is part of switcher-resample.
 *
 * switcher-resample is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "./resample.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Resample,
                                     "resample",
                                     "Audio resampler",
                                     "audio",
                                     "reader/writer",
                                     "Audio resampling with libsamplerate",
                                     "GPL",
                                     "Nicolas Bouillot");

Resample::Resample(quid::Config&& conf)
    : Quiddity(std::forward<quid::Config>(conf)),
      cntr_(static_cast<Quiddity*>(this)),
      samplerate_id_(
          pmanage<MPtr(&PContainer::make_parented_unsigned_int)>("samplerate",
                                                                 "",
                                                                 [this](unsigned int val) {
                                                                   samplerate_ = val;
                                                                   return true;
                                                                 },
                                                                 [this]() { return samplerate_; },
                                                                 "Samplerate",
                                                                 "The Samplerate to apply",
                                                                 samplerate_,
                                                                 1,
                                                                 192000)),
      algo_id_(pmanage<MPtr(&PContainer::make_selection<int>)>("algo",
                                                               [this](const IndexOrName& val) {
                                                                 algo_.select(val);
                                                                 return true;
                                                               },
                                                               [this]() { return algo_.get(); },
                                                               "Resampling algorithm",
                                                               "Balance fidelity vs. speed",
                                                               algo_)) {
  register_writer_suffix("audio");
  cntr_.install_connect_method([this](const std::string& shmpath) { return connect(shmpath); },
                               [this](const std::string&) { return disconnect(); },
                               [this]() { return disconnect(); },
                               [this](const std::string& caps) { return can_sink_caps(caps); },
                               1);
  resampled_.resize(resampled_size_);
  in_converted_.resize(resampled_size_);
}

bool Resample::connect(const std::string& path) {
  shmr_.reset();
  shmw_.reset();
  pmanage<MPtr(&PContainer::disable)>(algo_id_, ShmdataConnector::disabledWhenConnectedMsg);
  pmanage<MPtr(&PContainer::disable)>(samplerate_id_, ShmdataConnector::disabledWhenConnectedMsg);
  shmr_ = std::make_unique<ShmdataFollower>(
      this,
      path,
      [this](void* buf, size_t size) {
        // configure resampler
        resampler_data_->input_frames =
            size / (incaps_->channels() * incaps_->format_size_in_bytes());
        resampler_data_->output_frames = resampler_data_->input_frames * resampler_data_->src_ratio;
        // resize buffers if necessary
        auto expected_resampled_size =
            resampler_data_->output_frames * incaps_->channels() * sizeof(float);
        if (resampled_size_ < expected_resampled_size) {
          resampled_size_ = expected_resampled_size;
          resampled_.resize(resampled_size_);
        }
        auto required_sample_for_convertion =
            size * sizeof(float) / incaps_->format_size_in_bytes();
        if (in_converted_size_ < required_sample_for_convertion) {
          in_converted_size_ = required_sample_for_convertion;
          in_converted_.resize(in_converted_size_);
        }
        // configure resampler memory
        if (incaps_->is_float()) {
          resampler_data_->data_in = static_cast<float*>(buf);
          resampler_data_->data_out = resampled_.data();
        } else if (incaps_->format_size_in_bytes() == sizeof(short)) {
          src_short_to_float_array(static_cast<short*>(buf),
                                   in_converted_.data(),
                                   size / incaps_->format_size_in_bytes());
          resampler_data_->data_in = in_converted_.data();
          resampler_data_->data_out = resampled_.data();
        } else if (incaps_->format_size_in_bytes() == sizeof(int)) {
          src_int_to_float_array(
              static_cast<int*>(buf), in_converted_.data(), size / incaps_->format_size_in_bytes());
          resampler_data_->data_in = in_converted_.data();
          resampler_data_->data_out = resampled_.data();
        } else {
          warning("BUG in resample shmr data handler");
          return;
        }
        // do resampling
        auto error = src_process(resampler_config_, resampler_data_.get());
        if (error) {
          warning("resample error:%", std::string(src_strerror(error)));
          return;
        }
        // write to shmdata
        auto written = resampler_data_->output_frames_gen * incaps_->channels() * sizeof(float);
        shmw_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(resampled_.data(), written);
        shmw_->bytes_written(written);
      },
      [this](const std::string& str_caps) {
        if (nullptr != resampler_config_) src_delete(resampler_config_);  // FIXME make this RAII

        incaps_ = std::make_unique<AudioCaps>(str_caps);
        if (!(*incaps_)) {
          warning("resample does not understand shmdata caps: %", incaps_->error_msg());
          return;
        }

        int error;
        resampler_config_ = src_new(algo_.get_attached(), incaps_->channels(), &error);
        if (nullptr == resampler_config_) {
          warning("resample quiddity could not intialize the resample: %",
                  std::string(src_strerror(error)));
        };
        resampler_data_ = std::make_unique<SRC_DATA>();
        resampler_data_->end_of_input = 0;
        resampler_data_->input_frames = 0;
        resampler_data_->src_ratio = (1.0 * samplerate_) / incaps_->samplerate();

        auto newcaps = *incaps_;
        newcaps.set_samplerate(samplerate_);
        newcaps.set_float();
        shmw_ = std::make_unique<ShmdataWriter>(this, make_shmpath("audio"), 1, newcaps.get());
      },
      nullptr);
  return true;
}

bool Resample::disconnect() {
  shmr_.reset();
  shmw_.reset();
  pmanage<MPtr(&PContainer::enable)>(algo_id_);
  pmanage<MPtr(&PContainer::enable)>(samplerate_id_);
  return true;
}

bool Resample::can_sink_caps(const std::string& str_caps) {
  auto audiocaps = AudioCaps(str_caps);
  if (!audiocaps) return false;
  if (!audiocaps.is_float() && audiocaps.format_size_in_bytes() != sizeof(short) &&
      audiocaps.format_size_in_bytes() != sizeof(int))
    return false;
  return true;
}

}  // namespace switcher
