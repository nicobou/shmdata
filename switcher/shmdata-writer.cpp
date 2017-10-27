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

#include "./shmdata-writer.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {

ShmdataWriter::ShmdataWriter(Quiddity* quid,
                             const std::string& path,
                             size_t memsize,
                             const std::string& data_descr)
    : quid_(quid),
      shmpath_(path),
      data_type_(data_descr),
      shmlog_(quid->get_log_ptr()),
      shm_(shmpath_, memsize, data_type_, &shmlog_),
      task_(shm_ ? std::make_unique<PeriodicTask<>>([this]() { this->update_quid_stats(); },
                                                    ShmdataStat::kDefaultUpdateInterval)
                 : nullptr) {
  if (shm_ && nullptr != quid_)
    quid_->graft_tree(
        ".shmdata.writer." + shmpath_,
        ShmdataUtils::make_tree(data_type_, ShmdataUtils::get_category(data_type_), ShmdataStat()));
}

ShmdataWriter::~ShmdataWriter() {
  if (shm_ && nullptr != quid_) quid_->prune_tree(std::string(".shmdata.writer.") + shmpath_);
}

void ShmdataWriter::bytes_written(size_t size) {
  std::unique_lock<std::mutex> lock(bytes_mutex_);
  shm_stats_.count_buffer(size);
}

void ShmdataWriter::update_quid_stats() {
  decltype(shm_stats_) stats;
  {
    std::unique_lock<std::mutex> lock(bytes_mutex_);
    stats = shm_stats_;
    shm_stats_.reset();
  }
  ShmdataStat::make_tree_updater(quid_, ".shmdata.writer." + shmpath_)(stats);
}

}  // namespace switcher
