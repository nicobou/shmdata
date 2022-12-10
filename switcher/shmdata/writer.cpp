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

#include "./writer.hpp"
#include "./caps/utils.hpp"

namespace switcher {
namespace shmdata {

Writer::Writer(quiddity::Quiddity* quid,
               const std::string& path,
               size_t memsize,
               const std::string& data_descr)
    : quid_(quid),
      shmpath_(path),
      data_type_(data_descr),
      shmlog_(quid),
      shm_(shmpath_, memsize, data_type_, &shmlog_),
      task_(shm_ ? std::make_unique<PeriodicTask<>>([this]() { this->update_quid_stats(); },
                                                    Stat::kDefaultUpdateInterval)
                 : nullptr) {
  if (shm_ && nullptr != quid_) {
    auto parent_path = ".shmdata.writer." + shmpath_;
    auto tree = quid_->prune_tree(parent_path, false);
    quid_->graft_tree(parent_path, quid_->get_shm_information_template(), false);
    if (tree) {
      for (auto& it : tree->get_child_keys(".")) {
        quid_->graft_tree(parent_path + "." + it, tree->prune(it), false);
      }
    }

    // Add Switcher and quiddity info in shmdata caps
    auto caps = data_type_ + ", " + quid_->get_quiddity_caps();

    quid_->graft_tree(parent_path + ".caps", InfoTree::make(caps), false);
    quid_->graft_tree(
        parent_path + ".category", InfoTree::make(caps::get_category(caps)), false);
    quid_->notify_tree_updated(parent_path);
  }
}

Writer::~Writer() {
  if (shm_ && nullptr != quid_) quid_->prune_tree(std::string(".shmdata.writer.") + shmpath_);
}

void Writer::bytes_written(size_t size) {
  std::unique_lock<std::mutex> lock(bytes_mutex_);
  shm_stats_.count_buffer(size);
}

void Writer::update_quid_stats() {
  decltype(shm_stats_) stats;
  {
    std::unique_lock<std::mutex> lock(bytes_mutex_);
    stats = shm_stats_;
    shm_stats_.reset();
  }
  Stat::make_tree_updater(quid_, ".shmdata.writer." + shmpath_)(stats);
}

}  // namespace shmdata
}  // namespace switcher
