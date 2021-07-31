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
#include "../utils/scope-exit.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmDelay,
                                     "shmdelay",
                                     "Shmdata delay line",
                                     "other",
                                     "reader/writer",
                                     "Takes an input shmdata and delays it by a fixed amount",
                                     "LGPL",
                                     "Jérémie Soria");

const std::string ShmDelay::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "shm",
      "description": "Stream to add delay to. This stream can be however limited to audio or video only with the restrict_caps property",
      "can_do": ["all"]
    },
    {
      "label": "ltf-diff",
      "description": "Stream from ltf-diff quiddity, used to control de delay to apply",
      "can_do": ["audio/ltc-diff"]
    }
  ],
"writer":
  [
    {
      "label": "delayed-shm",
      "description": "Stream with delay",
      "can_do": ["all"]
    }
  ]
}
)");

ShmDelay::ShmDelay(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t sfid) {
                  return on_shmdata_connect(shmpath, sfid);
                },
                [this](claw::sfid_t sfid) { return on_shmdata_disconnect(sfid); }}),
      time_delay_id_(pmanage<MPtr(&property::PBag::make_double)>(
          "time_delay",
          [this](const double& val) {
            time_delay_ = val;
            return true;
          },
          [this]() { return time_delay_; },
          "Delay in ms",
          "Delay the input shmdata by a delay in ms",
          time_delay_,
          0,
          60000)),
      buffer_size_id_(pmanage<MPtr(&property::PBag::make_unsigned_int)>(
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
          8096)),
      restrict_caps_id_(pmanage<MPtr(&property::PBag::make_selection<>)>(
          "restrict_caps",
          [this](const quiddity::property::IndexOrName& val) {
            restrict_caps_.select(val);
            auto index = restrict_caps_.get_current_index();
            if (index == 0) {
              claw_.replace_follower_can_do(claw_.get_sfid("shm"), {::shmdata::Type("all")});
            } else if (index == 1) {
              claw_.replace_follower_can_do(claw_.get_sfid("shm"),
                                            {::shmdata::Type("audio/x-raw")});
            } else if (index == 2) {
              claw_.replace_follower_can_do(claw_.get_sfid("shm"),
                                            {::shmdata::Type("video/x-raw")});
            } else if (index == 3) {
              claw_.replace_follower_can_do(
                  claw_.get_sfid("shm"),
                  {::shmdata::Type("audio/x-raw"), ::shmdata::Type("video/x-raw")});
            }
            return true;
          },
          [this]() { return restrict_caps_.get(); },
          "Restrict authorized caps",
          "Input data capabilities must comply with the selected caps type (None means any)",
          restrict_caps_)) {}

bool ShmDelay::on_shmdata_connect(const std::string& shmpath, claw::sfid_t sfid) {
  // Get the value of the delay from a shmdata
  if (claw_.get_follower_label(sfid) == "ltc-diff") {
    if (diff_follower_) diff_follower_.reset();
    diff_follower_ = std::make_unique<shmdata::Follower>(
        this,
        shmpath,
        [this](void* data, size_t data_size) {
          if (!data_size) return;
          pmanage<MPtr(&property::PBag::set<double>)>(time_delay_id_, *static_cast<double*>(data));
        },
        nullptr,
        nullptr,
        shmdata::Stat::kDefaultUpdateInterval,
        shmdata::Follower::Direction::reader,
        true);
    pmanage<MPtr(&property::PBag::disable)>(
        time_delay_id_, "Delay is provided by an ltc-diff shmdata, it cannot be changed manually");
  } else {
    // We do not delay using an ltc-diff shmdata.
    if (shm_follower_) {
      writing_task_.reset();
      shmw_.reset();
      shm_follower_.reset();
    }

    // Follow the shmdata to delay.
    shm_follower_ = std::make_unique<shmdata::Follower>(
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
          shmw_ = std::make_unique<shmdata::Writer>(
              this, claw_.get_shmpath_from_writer_label("delayed-shm"), 1, str_caps);

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

                shmw_->writer<MPtr(&::shmdata::Writer::copy_to_shm)>(closest.content_.data(),
                                                                     closest.data_size_);
                shmw_->bytes_written(closest.data_size_);
              },
              std::chrono::microseconds(100));
        },
        nullptr,
        shmdata::Stat::kDefaultUpdateInterval,
        shmdata::Follower::Direction::reader,
        true);
  }

  return true;
}

bool ShmDelay::on_shmdata_disconnect(claw::sfid_t sfid) {
  if (claw_.get_follower_label(sfid) == "ltc-diff") {
    diff_follower_.reset();
    pmanage<MPtr(&property::PBag::enable)>(time_delay_id_);
  } else {
    writing_task_.reset();
    shmw_.reset();
    shm_follower_.reset();
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

}  // namespace quiddities
}  // namespace switcher
