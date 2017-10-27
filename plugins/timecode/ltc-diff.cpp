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

#include "ltc-diff.hpp"
#include "switcher/gst-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(LTCDiff,
                                     "ltcdiff",
                                     "LTC timecode difference",
                                     "time",
                                     "reader/writer",
                                     "Computes the absolute time between two timecode shmdata",
                                     "LGPL",
                                     "Jérémie Soria");

LTCDiff::LTCDiff(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  display_timecode1_id_ =
      pmanage<MPtr(&PContainer::make_string)>("first_timecode",
                                              nullptr,
                                              [this]() { return display_timecodes_.at(0); },
                                              "Timecode 1",
                                              "Timecode of the first connected source",
                                              display_timecodes_.at(0));

  display_timecode2_id_ =
      pmanage<MPtr(&PContainer::make_string)>("second_timecode",
                                              nullptr,
                                              [this]() { return display_timecodes_.at(1); },
                                              "Timecode 2",
                                              "Timecode of the second connected source",
                                              display_timecodes_.at(1));

  notify_task_ = std::make_unique<PeriodicTask<>>(
      [this]() {
        pmanage<MPtr(&PContainer::get_lock)>(display_timecode1_id_);
        pmanage<MPtr(&PContainer::notify)>(display_timecode1_id_);
        pmanage<MPtr(&PContainer::get_lock)>(display_timecode2_id_);
        pmanage<MPtr(&PContainer::notify)>(display_timecode2_id_);
      },
      std::chrono::milliseconds(500));

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string& shmpath) { return this->on_shmdata_disconnect(shmpath); },
      nullptr,
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      2);
}

bool LTCDiff::on_shmdata_connect(const std::string& shmpath) {
  ltc_readers_[shmpath] = std::make_unique<LTCReader>(this, shmpath, next_index_);
  next_index_ = next_index_ ? 0 : 1;

  if (!shm_follower_) {
    shmw_ = std::make_unique<ShmdataWriter>(this, make_file_name("ltc-diff"), 1, "audio/ltc-diff");
    shm_follower_ = std::make_unique<ShmdataFollower>(
        this,
        shmpath,
        [this](void*, size_t) {
          std::lock_guard<std::mutex> lock(timecode_m_);
          if (!do_compute_) {
            if (ltc_readers_.size() != 2) return;
            for (auto& reader : ltc_readers_) {
              if (reader.second->timecode_ == 0) return;
            }
            do_compute_ = true;
          }
          time_difference_ = 0;
          for (auto& reader : ltc_readers_) {
            // If either timecode is null, do not compute the difference
            if (!reader.second->timecode_) return;

            // We consider the timecodes are far enough that we do not need to know which one is
            // late and which one is early, we do not target sub-frame precision drift correction.
            if (!time_difference_)
              time_difference_ += reader.second->timecode_;
            else
              time_difference_ -= reader.second->timecode_;

            reader.second->timecode_ = 0;
          }

          time_difference_ = std::max<double>(time_difference_, -time_difference_) * 1000;
          shmw_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(&time_difference_,
                                                             sizeof(time_difference_));
          shmw_->bytes_written(sizeof(time_difference_));
          time_difference_ = 0;
        },
        nullptr,
        nullptr);
  }
  return true;
}

bool LTCDiff::on_shmdata_disconnect(const std::string& shmpath) {
  std::lock_guard<std::mutex> lock(timecode_m_);
  auto reader = ltc_readers_.find(shmpath);
  if (reader != ltc_readers_.end()) {
    ltc_readers_.erase(reader);
    do_compute_ = false;
  }

  return true;
}

bool LTCDiff::can_sink_caps(const std::string& str_caps) {
  return GstUtils::can_sink_caps("audioconvert", str_caps);
}

LTCDiff::LTCReader::LTCReader(LTCDiff* quid, const std::string& shmpath, size_t index)
    : parent_(quid), index_(index) {
  decoder_ = ltc_decoder_create(1920, 32);
  if (nullptr == decoder_) return;

  shm_follower_ = std::make_unique<ShmdataFollower>(
      quid,
      shmpath,
      [this](void* data, size_t data_size) { on_data(data, data_size); },
      nullptr,
      nullptr);
}

LTCDiff::LTCReader::~LTCReader() {
  if (decoder_) ltc_decoder_free(decoder_);
}

void LTCDiff::LTCReader::on_data(void* data, size_t data_size) {
  auto samples = static_cast<uint8_t*>(data);

  // Do not try detcting ltc if the buffer only contains silence
  bool silence = true;
  for (unsigned int i = 0; i < data_size; ++i) {
    auto sample = samples[i];
    if (silence && (sample < 126 || sample > 129)) {
      silence = false;
      break;
    }
  }

  if (silence) return;

  ltc_decoder_write(decoder_, samples, data_size, 0);

  LTCFrameExt ltc_frame;
  double timecode_secs = 0;
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

    timecode_secs =
        (tc.hours * 3600 + tc.mins * 60 + tc.secs) + tc.frame / static_cast<double>(ltc_fps_);
    parent_->display_timecodes_.at(index_) =
        std::to_string(tc.hours) + ":" + std::to_string(tc.mins) + ":" + std::to_string(tc.secs) +
        "." + std::to_string(tc.frame);
  }

  std::lock_guard<std::mutex> lock(timecode_m_);
  if (timecode_secs) {
    timecode_ = timecode_secs;
  }
}

}  // namespace switcher
