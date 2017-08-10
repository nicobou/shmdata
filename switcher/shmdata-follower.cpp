/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#include "./shmdata-follower.hpp"
#include "./shmdata-utils.hpp"

namespace switcher {

ShmdataFollower::ShmdataFollower(Quiddity* quid,
                                 const std::string& path,
                                 shmdata::Reader::onData od,
                                 shmdata::Reader::onServerConnected osc,
                                 shmdata::Reader::onServerDisconnected osd,
                                 std::chrono::milliseconds update_interval,
                                 const std::string& tree_path)
    : quid_(quid),
      logger_(quid_->get_log_ptr()),
      shmpath_(path),
      od_(od),
      osc_(osc),
      osd_(osd),
      tree_path_(tree_path),
      follower_(std::make_unique<shmdata::Follower>(
          shmpath_,
          [this](void* data, size_t size) { this->on_data(data, size); },
          [this](const std::string& data_type) { this->on_server_connected(data_type); },
          [this]() { this->on_server_disconnected(); },
          &logger_)),
      task_(std::make_unique<PeriodicTask<>>([this]() { this->update_quid_stats(); },
                                             update_interval)) {}

ShmdataFollower::~ShmdataFollower() {
  follower_.reset(nullptr);
  if (!data_type_.empty()) quid_->prune_tree(tree_path_ + shmpath_);
}

void ShmdataFollower::on_data(void* data, size_t size) {
  {
    std::unique_lock<std::mutex> lock(bytes_mutex_);
    shm_stat_.count_buffer(size);
  }
  if (!od_) return;
  od_(data, size);
}

void ShmdataFollower::on_server_connected(const std::string& data_type) {
  if (data_type != data_type_) {
    data_type_ = data_type;
    quid_->graft_tree(
        tree_path_ + shmpath_,
        ShmdataUtils::make_tree(data_type_, ShmdataUtils::get_category(data_type_), ShmdataStat()));
  }
  if (osc_) osc_(data_type);
}

void ShmdataFollower::on_server_disconnected() {
  if (osd_) osd_();
}

void ShmdataFollower::update_quid_stats() {
  std::unique_lock<std::mutex> lock(bytes_mutex_);
  ShmdataStat::make_tree_updater(quid_, tree_path_ + shmpath_)(shm_stat_);
  shm_stat_.reset();
}

}  // namespace switcher
