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

#include "./shm-delay.hpp"
#include "./scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmDelay,
                                     "shmdelay",
                                     "Shmdata delay line",
                                     "other",
                                     "reader/writer",
                                     "Takes an input shmdata and delays it by a fixed amount",
                                     "LGPL",
                                     "Jérémie Soria");

ShmDelay::ShmDelay(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  time_delay_id_ =
      pmanage<MPtr(&PContainer::make_double)>("time_delay",
                                              [this](const double& val) {
                                                time_delay_ = val;
                                                return true;
                                              },
                                              [this]() { return time_delay_; },
                                              "Delay in ms",
                                              "Delay the input shmdata by a delay in ms",
                                              time_delay_,
                                              0,
                                              60000);

  pmanage<MPtr(&PContainer::make_unsigned_int)>(
      "buffer_size",
      [this](const unsigned int& val) {
        buffer_size_ = val;
        delay_content_.set_buffer_size(buffer_size_);
        return true;
      },
      [this]() { return buffer_size_; },
      "Maximum physical size of the buffer (in MB)",
      "Maximum size taken by the data buffer in physical memory in MB",
      buffer_size_,
      128,  // Minimum 100 MB in the buffer otherwise it doesn't really make sense
      8096);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string& shmpath) { return this->on_shmdata_disconnect(shmpath); },
      nullptr,
      [this](const std::string&) { return true; },
      2);
}

bool ShmDelay::on_shmdata_connect(const std::string& shmpath) {
  // Get the value of the delay from a shmdata
  if (!diff_follower_ && StringUtils::ends_with(shmpath, "ltc-diff")) {
    diff_follower_ = std::make_unique<ShmdataFollower>(
        this,
        shmpath,
        [this](void* data, size_t data_size) {
          if (!data_size) return;
          pmanage<MPtr(&PContainer::set<double>)>(time_delay_id_, *static_cast<double*>(data));
        },
        nullptr,
        nullptr);
    pmanage<MPtr(&PContainer::disable)>(
        time_delay_id_, "Delay is provided by an ltc-diff shmdata, it cannot be changed manually");
    return true;
  }

  // We do not delay an ltc-diff shmdata.
  if (shm_follower_ || StringUtils::ends_with(shmpath, "ltc-diff")) return false;

  // Follow the shmdata to delay.
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
              auto closest = delay_content_.find_closest(current_time - time_delay_);

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

bool ShmDelay::on_shmdata_disconnect(const std::string& shmpath) {
  if (StringUtils::ends_with(shmpath, "ltc-diff")) {
    diff_follower_.reset(nullptr);
    pmanage<MPtr(&PContainer::enable)>(time_delay_id_);
  } else {
    writing_task_.reset(nullptr);
    shmw_.reset(nullptr);
    shm_follower_.reset(nullptr);
  }

  return true;
}

ShmDelay::ShmContent::ShmContent(double timestamp, void* content, size_t data_size)
    : data_size_(data_size), timestamp_(timestamp) {
  content_.resize(data_size);

  // We serialize the shmdata into a vector
  memcpy(content_.data(), content, data_size);
}

void ShmDelay::ShmBuffer::push(const ShmContent& content) {
  std::lock_guard<std::mutex> lock(buffer_m_);
  buffer_.push_back(content);
  total_size_ += content.get_size();

  // If the new sample made us go over the buffer physical limit, we remove frames until they all
  // fit the buffer
  while (total_size_ > max_size_) {
    total_size_ -= buffer_.front().get_size();
    buffer_.pop_front();
  }
}

void ShmDelay::ShmBuffer::set_buffer_size(const size_t& size) {
  std::lock_guard<std::mutex> lock(buffer_m_);
  max_size_ = size * (1 << 20);  // Convert megabytes to bytes.

  // Remove older frames until they all fit the new size
  while (total_size_ > max_size_) {
    total_size_ -= buffer_.front().get_size();
    buffer_.pop_front();
  }
}

ShmDelay::ShmContent ShmDelay::ShmBuffer::find_closest(double target_timestamp) const {
  // Arbitrary initialization of the difference between the target timestamp and the frames from the
  // buffer
  double diff = 1000;

  // Used to detect that we are getting closer to the target timestamp
  bool getting_closer = false;

  std::lock_guard<std::mutex> lock(buffer_m_);
  if (buffer_.empty()) return ShmContent();
  auto closest = buffer_.front();
  for (auto& item : buffer_) {
    // We take the absolute value of the difference with the
    auto new_diff = item.timestamp_ - target_timestamp;
    if (new_diff < 0) new_diff = -new_diff;
    // We are getting closer to the target timestamp
    if (new_diff < diff) {
      diff = new_diff;
      closest = item;
      getting_closer = true;
    } else if (getting_closer && new_diff > diff) {
      // We were getting closer and now getting further, so we will return the previous item.
      break;
    }
  }

  return closest;
}

}  // namespace switcher
