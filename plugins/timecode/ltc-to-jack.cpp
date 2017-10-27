/*
 * This file is part of switcher-timecode.
 *
 * switcher-timecode is free software; you can redistribute it and/or
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

#include "ltc-to-jack.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(LTCToJack,
                                     "ltctojack",
                                     "LTC to Jack Transport",
                                     "audio",
                                     "reader",
                                     "Control Jack Transport from an incoming LTC stream",
                                     "LGPL",
                                     "Jérémie Soria");

LTCToJack::LTCToJack(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  jack_client_ = jack_client_open(
      std::string(std::string("clockLTC_") + get_name()).c_str(), JackNullOption, nullptr);

  if (!jack_client_) {
    warning("Could not create jack client (ltctojack).");
    is_valid_ = false;
    return;
  }

  if (jack_activate(jack_client_) != 0) {
    warning("Could not activate jack client (ltctojack).");
    is_valid_ = false;
    return;
  }

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return on_shmdata_connect(shmpath); },
      [this](const std::string&) { return on_shmdata_disconnect(); },
      [this]() { return on_shmdata_disconnect(); },
      [this](const std::string& caps) { return can_sink_caps(caps); },
      1);

  drift_threshold_id_ = pmanage<MPtr(&PContainer::make_double)>(
      "drift_threshold",
      [this](const double& val) {
        std::lock_guard<std::mutex> lock(threshold_mutex_);
        drift_threshold_ = val;
        return true;
      },
      [this]() { return drift_threshold_; },
      "Minimum clock difference threshold",
      "Minimum clock difference threshold before forcing repositining of jack transport",
      drift_threshold_,
      0.01,
      10.);
}

LTCToJack::~LTCToJack() {
  if (is_valid_) {
    jack_deactivate(jack_client_);
    jack_client_close(jack_client_);
  }
}

bool LTCToJack::on_shmdata_connect(const std::string& shmpath) {
  shm_follower_ = std::make_unique<ShmdataFollower>(
      this, shmpath, [this](void* data, size_t size) { on_data(data, size); }, nullptr, nullptr);

  jack_transport_start(jack_client_);
  jack_transport_locate(jack_client_, 0);

  // The audio frame per video frame value 1920 has been arbitrarily chosen but will be set by the
  // decoder automatically
  decoder_ = ltc_decoder_create(1920, 32);
  return decoder_ != nullptr;
}

bool LTCToJack::on_shmdata_disconnect() {
  jack_transport_stop(jack_client_);
  jack_time_ref_ = 0;
  shm_follower_.reset();
  ltc_decoder_free(decoder_);
  return true;
}

void LTCToJack::on_data(void* data, size_t data_size) {
  auto samples = static_cast<float*>(data);
  auto nb_samples = data_size / sizeof(float);

  // Using the float samples with the ltc decoder leads to glitches in the timecode so we work with
  // unsigned char instead
  std::vector<uint8_t> u8_samples;
  bool silence = true;
  for (unsigned int i = 0; i < nb_samples; ++i) {
    auto sample = static_cast<uint8_t>(samples[i] * 127.5 + 127.5);
    if (silence && (sample < 126 || sample > 129)) silence = false;
    u8_samples.push_back(sample);
  }

  if (silence && !paused_) {
    jack_transport_stop(jack_client_);
    paused_ = true;
  } else if (!silence && paused_) {
    jack_transport_start(jack_client_);
    jack_time_ref_ = 0.;  // Force recomputing of clock difference after pause/play
    paused_ = false;
    force_locate_ = true;
  }

  if (silence) return;

  ltc_decoder_write(decoder_, u8_samples.data(), nb_samples, 0);

  // offset_ += data_size;

  jack_position_t pos;
  jack_transport_query(jack_client_, &pos);

  LTCFrameExt ltc_frame;
  double current_ltc_time = 0.;
  while (!silence && ltc_decoder_read(decoder_, &ltc_frame)) {
    SMPTETimecode tc;
    ltc_frame_to_time(&tc, &ltc_frame.ltc, LTC_USE_DATE);

    // LTC fps detection
    if (!ltc_fps_detected_) {
      if (!ltc_first_zero_detected && tc.frame == 0) {
        ltc_first_zero_detected = true;
      } else if (ltc_first_zero_detected) {
        if (tc.frame == 0) {
          ltc_fps_ = tmp_ltc_fps_ + 1;
          ltc_fps_detected_ = true;
        } else
          tmp_ltc_fps_ = tc.frame;
      }
    }

    current_ltc_time =
        (tc.hours * 3600 + tc.mins * 60 + tc.secs) + tc.frame / static_cast<double>(ltc_fps_);

    auto current_jack_time = static_cast<double>(pos.usecs) / 1000000.;

    if (jack_time_ref_ < std::numeric_limits<double>::epsilon()) {
      jack_time_ref_ = current_jack_time;
      diff_clocks_ = jack_time_ref_ - current_ltc_time;
    }

    double time_delta = current_jack_time - current_ltc_time - diff_clocks_;
    if (time_delta < 0) time_delta = -time_delta;

    {
      std::lock_guard<std::mutex> lock(threshold_mutex_);
      if (force_locate_ || time_delta > drift_threshold_) {
        jack_transport_locate(jack_client_,
                              static_cast<jack_nframes_t>(current_ltc_time * pos.frame_rate));
        jack_time_ref_ = 0.;  // Force recomputing of clock difference after repositioning
        if (force_locate_) force_locate_ = false;
      }
    }
  }
}

bool LTCToJack::can_sink_caps(std::string str_caps) {
  return StringUtils::starts_with(str_caps, "audio/x-raw");
}

}  // namespace switcher
