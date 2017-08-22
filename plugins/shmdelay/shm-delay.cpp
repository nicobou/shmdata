/*
 * This file is part of switcher-shmdelay.
 *
 * switcher-shmdelay is free software; you can redistribute it and/or
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

#include "shm-delay.hpp"
#include "switcher/scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    ShmDelay,
    "shmdelay",
    "Shmdata audio/video delay line",
    "audio-video",
    "reader/writer",
    "Takes an input audio/video shmdata and delays it by a fixed amount",
    "LGPL",
    "Jérémie Soria");

ShmDelay::ShmDelay(const std::string&) : shmcntr_(static_cast<Quiddity*>(this)) {
  pmanage<MPtr(&PContainer::make_unsigned_int)>("time_delay",
                                                [this](const unsigned int& val) {
                                                  time_delay_ = val;
                                                  return true;
                                                },
                                                [this]() { return time_delay_; },
                                                "Delay the input shmdata by a delay in ms",
                                                "Bla bla",
                                                time_delay_,
                                                0,
                                                10000);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      3);

  is_valid_ = true;
}

bool ShmDelay::init() { return is_valid_; }

ShmDelay::~ShmDelay() {}

bool ShmDelay::on_shmdata_connect(const std::string& shmpath) {
  shm_follower_ = std::make_unique<ShmdataFollower>(
      this,
      shmpath,
      [this](void* data, size_t data_size) {
        if (!data_size) return;

        // Push the shmdata in the ShmBuffer with the current timestamp.
        auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
        delay_content_.push(ShmContent(static_cast<double>(current_time), data, data_size));
      },
      [this](const std::string& str_caps) {
        GstCaps* caps = gst_caps_from_string(str_caps.c_str());
        On_scope_exit {
          if (nullptr != caps) gst_caps_unref(caps);
        };

        // We are only delaying so the caps will be identical to the received shmdata
        shmw_ = std::make_unique<ShmdataWriter>(this, make_file_name("delayed-shm"), 1, str_caps);

        // Task checking if a previously recorded shmdata is a candidate to be written.
        writing_task_ = std::make_unique<PeriodicTask<std::chrono::microseconds>>(
            [this]() {
              auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count();
              // Look for the closest shmdata content in time to our target time
              auto closest = delay_content_.find(current_time - time_delay_);

              if (!closest.data_size_) return;

              // Same frame as before, we do not forward it.
              if (closest.timestamp_ == last_timestamp_) return;

              // We record the timestamp of the shmdata we are now forwarding.
              last_timestamp_ = closest.timestamp_;

              shmw_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(closest.content_.data(),
                                                                 closest.data_size_);
              shmw_->bytes_written(closest.data_size_);
            },
            std::chrono::microseconds(100));
      },
      nullptr);

  return true;
}

bool ShmDelay::on_shmdata_disconnect() {
  shm_follower_.reset(nullptr);
  writing_task_.reset(nullptr);
  shmw_.reset(nullptr);
  return true;
}

bool ShmDelay::can_sink_caps(const std::string& str_caps) {
  return GstUtils::can_sink_caps("audioconvert", str_caps) ||
         GstUtils::can_sink_caps("videoconvert", str_caps);
}

ShmDelay::ShmContent::ShmContent(double timestamp, void* content, size_t data_size)
    : data_size_(data_size), timestamp_(timestamp) {
  auto current_content = static_cast<unsigned char*>(content);
  for (size_t i = 0; i < data_size_ / sizeof(unsigned char); ++i) {
    content_.push_back(current_content[i]);
  }
}

void ShmDelay::ShmBuffer::push(const ShmContent& content) {
  std::lock_guard<std::mutex> lock(buffer_m_);
  buffer_.push_back(content);
  total_size += content.get_size();
  if (total_size > max_size) {
    total_size -= buffer_.front().get_size();
    buffer_.pop_front();
  }
}

ShmDelay::ShmContent ShmDelay::ShmBuffer::find(double target_timestamp) const {
  double diff = 1000;
  bool getting_closer = false;

  std::lock_guard<std::mutex> lock(buffer_m_);
  auto closest = buffer_.front();
  for (auto& item : buffer_) {
    auto new_diff = std::abs(item.timestamp_ - target_timestamp);
    if (new_diff < diff) {
      diff = new_diff;
      closest = item;
      getting_closer = true;
    } else if (getting_closer && new_diff > diff && diff < 10) {
      break;
    }
  }

  return closest;
}

}  // namespace switcher
