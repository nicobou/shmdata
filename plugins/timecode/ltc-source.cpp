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

#include "ltc-source.hpp"
#include "switcher/scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    LTCSource,
    "ltcsource",
    "LTC timecode source",
    "audio",
    "reader/writer",
    "Generate an LTC timecode audio track (can be cadenced by external sound shmdata)",
    "LGPL",
    "Jérémie Soria");

LTCSource::LTCSource(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  jack_client_ = jack_client_open(
      std::string(std::string("genLTC_") + get_name()).c_str(), JackNullOption, nullptr);

  if (!jack_client_) {
    warning("Could not create jack client (ltcsource).");
    is_valid_ = false;
    return;
  }

  sample_rate_ = jack_get_sample_rate(jack_client_);
  if (!sample_rate_) {
    warning("Could not get sample rate from jack server (ltcsource).");
    message("ERROR: Could not get sample rate from jack server (ltcsource).");
    is_valid_ = false;
    return;
  }

  time_reference_id_ = pmanage<MPtr(&PContainer::make_selection<>)>(
      "time_reference",
      [this](const IndexOrName& val) {
        time_reference_.select(val);
        return true;
      },
      [this]() { return time_reference_.get(); },
      "Time reference",
      "Select the time reference for the generated timecode",
      time_reference_);

  fps_id_ = pmanage<MPtr(&PContainer::make_selection<double>)>(
      "fps",
      [this](const IndexOrName& val) {
        fps_.select(val);
        return true;
      },
      [this]() { return fps_.get(); },
      "Target FPS",
      "Desired frame per second setting for the encoder.",
      fps_);

  timeshift_fw_id_ = pmanage<MPtr(&PContainer::make_unsigned_int)>(
      "timeshift_forward",
      [this](const unsigned int& val) {
        // We increment the initial timecode.
        timeshift_fw_ = val;
        return true;
      },
      [this]() { return timeshift_fw_; },
      "Delay to add at the start of the generated source",
      "Shift the starting timecode by a fixed amount of frames.",
      timeshift_fw_,
      0,
      1000);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
  init_startable(this);
}

LTCSource::~LTCSource() {
  if (is_started()) pmanage<MPtr(&PContainer::set_str_str)>("started", "false");
  jack_client_close(jack_client_);
}

void LTCSource::tick_callback(jack_nframes_t nframes) {
  // Always keep 5 loops worth of samples in advance to avoid delays.
  if (!generating_frames_ && samples_.size() < nframes * 5)
    generation_th_->run_async(
        [this, nframes]() { generate_ltc_frames(static_cast<int>(sample_rate_ / nframes) * 5); });
  write_samples_to_shmdata(static_cast<unsigned int>(nframes));
}

bool LTCSource::start() {
  shmw_ = std::make_unique<ShmdataWriter>(
      this,
      make_file_name("audio"),
      1,
      std::string(
          "audio/"
          "x-raw,layout=(string)interleaved,channels=(int)1,format=(string)S16LE,rate=(int)") +
          std::to_string(sample_rate_));

  if (!external_sync_source_) {
    jack_set_process_callback(jack_client_,
                              [](jack_nframes_t nframes, void* arg) {
                                auto source = static_cast<LTCSource*>(arg);
                                source->tick_callback(nframes);
                                return 0;
                              },
                              this);
  }

  // We generate LTC frames, so we nee to compute the reference timecode depending on configuration
  // (wall clock, relative to start, timeshift)
  const char timezone[6] = "+0000";
  st_.frame = 0;

  if (time_reference_.get().index_ != 0) {
    strcpy(st_.timezone, timezone);
    st_.years = 0;
    st_.months = 1;
    st_.days = 1;
    st_.hours = 0;
    st_.mins = 0;
    st_.secs = 0;
  } else {
    time_t rawtime;
    time(&rawtime);
    struct tm* time_info;

    time_info = gmtime(&rawtime);
    st_.years = static_cast<unsigned char>(time_info->tm_year);
    st_.months = static_cast<unsigned char>(time_info->tm_mon);
    st_.days = static_cast<unsigned char>(time_info->tm_mday);
    st_.hours = static_cast<unsigned char>(time_info->tm_hour);
    st_.mins = static_cast<unsigned char>(time_info->tm_min);
    st_.secs = static_cast<unsigned char>(time_info->tm_sec);
  }

  encoder_ = ltc_encoder_create(sample_rate_,
                                fps_.get_attached(),
                                fps_.get_attached() == 25
                                    ? LTC_TV_625_50
                                    : LTC_TV_525_60,  // The other standards are equivalent, only
                                // 25 fps is an exception
                                LTC_USE_DATE);

  if (!encoder_) {
    warning("Failed to create LTC encoder (ltcsource).");
    message("ERROR: Failed to create LTC encoder (ltcsource).");
    return false;
  }

  ltc_encoder_set_timecode(encoder_, &st_);

  for (unsigned int i = 0; i < timeshift_fw_; ++i) ltc_encoder_inc_timecode(encoder_);

  // We generate a first batch of samples before activating the jack client to avoid "buffering".
  generate_ltc_frames(static_cast<int>(fps_.get_attached()) * 5);

  // Now we start generating or reading on the cadence provided by the jack client
  if (!external_sync_source_ && jack_activate(jack_client_) != 0) {
    warning("Could not activate jack client (ltcsource).");
    return false;
  }

  pmanage<MPtr(&PContainer::disable)>(time_reference_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(fps_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(timeshift_fw_id_, disabledWhenStartedMsg);

  return true;
}

bool LTCSource::stop() {
  ltc_encoder_free(encoder_);
  if (!external_sync_source_) jack_deactivate(jack_client_);
  shmw_.reset(nullptr);
  samples_.clear();

  pmanage<MPtr(&PContainer::enable)>(time_reference_id_);
  pmanage<MPtr(&PContainer::enable)>(fps_id_);
  pmanage<MPtr(&PContainer::enable)>(timeshift_fw_id_);

  return true;
}

bool LTCSource::on_shmdata_connect(const std::string& shmpath) {
  // We cannot connect a source while already generating LTC.
  if (is_started()) {
    message("ERROR: Cannot connect sound source for cadencing during LTC generation (ltcsource).");
    warning("Cannot connect sound source for cadencing during LTC generation (ltcsource).");
    return false;
  }

  shm_follower_ = std::make_unique<ShmdataFollower>(
      this,
      shmpath,
      [this](void*, size_t size) {
        // Only process samples if started.
        if (!is_started()) return;

        // We compute the number of jack frames in the data received.
        if (format_size_) tick_callback(static_cast<jack_nframes_t>(size / format_size_));
      },
      [this](const std::string& str_caps) {
        GstCaps* caps = gst_caps_from_string(str_caps.c_str());
        On_scope_exit {
          if (nullptr != caps) gst_caps_unref(caps);
        };

        GstStructure* s = gst_caps_get_structure(caps, 0);
        if (nullptr == s) {
          warning("Cannot get structure from caps (ltcsource)");
          return;
        }

        auto format = gst_structure_get_string(s, "format");
        if (!format) {
          warning("Cannot get format from shmdata description (ltcsource)");
          return;
        }

        int sample_rate = 0;
        if (!gst_structure_get_int(s, "rate", &sample_rate)) {
          warning("Cannot get rate from shmdata description (ltcsource)");
          return;
        }
        sample_rate_ = static_cast<unsigned int>(sample_rate);

        std::string str_format(format);
        str_format = str_format.substr(1, str_format.size());
        if (StringUtils::starts_with(str_format, "8"))
          format_size_ = 1;
        else if (StringUtils::starts_with(str_format, "16"))
          format_size_ = 2;
        else if (StringUtils::starts_with(str_format, "32"))
          format_size_ = 4;
        else if (StringUtils::starts_with(str_format, "64"))
          format_size_ = 8;
        else {
          format_size_ = 0;
          message(
              "ERROR: Only supports 8/16/32/64 bits audio formats for external sync "
              "source (ltcsource).");
          warning(
              "Only supports 8/16/32/64 bits audio formats for external sync source (ltcsource).");
        }
      },
      nullptr);

  external_sync_source_ = true;

  return true;
}

bool LTCSource::on_shmdata_disconnect() {
  shm_follower_.reset(nullptr);

  // We don't switch the source of the ticks during the generation so we stop if we get disconnected
  // from the shmdata.
  if (is_started()) {
    pmanage<MPtr(&PContainer::set_str_str)>("started", "false");
    message("ERROR: LTC generation stopped because the tick source was disconnected (ltcsource).");
    warning("LTC generation stopped because the tick source was disconnected (ltcsource).");
  }

  external_sync_source_ = false;

  return true;
}

bool LTCSource::can_sink_caps(const std::string& str_caps) {
  return GstUtils::can_sink_caps("audioconvert", str_caps);
}

void LTCSource::write_samples_to_shmdata(const unsigned int& nb_samples) {
  if (!nb_samples || samples_.size() < nb_samples) return;

  auto samples_size = sizeof(ltcsnd_sample_t) * nb_samples;
  std::vector<ltcsnd_sample_t> array;
  array.reserve(samples_size);
  std::copy(samples_.begin(), samples_.begin() + nb_samples, array.begin());
  shmw_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(array.data(), samples_size);
  shmw_->bytes_written(samples_size);
  for (unsigned int i = 0; i < nb_samples; ++i) {
    samples_.pop_front();
  }
}

void LTCSource::generate_ltc_frames(int nb_frames) {
  if (!encoder_) return;

  generating_frames_ = true;

  // Generate a batch of 5 seconds worth of samples
  for (int frame = 0; frame < nb_frames; ++frame) {
    ltcsnd_sample_t* buf;
    int len;
    ltc_encoder_encode_frame(encoder_);
    buf = ltc_encoder_get_bufptr(encoder_, &len, 1);
    for (int sample = 0; sample < len; ++sample) samples_.emplace_back(buf[sample]);
    ltc_encoder_inc_timecode(encoder_);
  }

  generating_frames_ = false;
}

}  // namespace switcher
