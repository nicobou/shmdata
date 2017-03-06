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

#include "./shmdata-stat.hpp"

namespace switcher {

// We use two for simplicity here, tasks need to be created with milliseconds and it is better for
// precision to have seconds for the display of the byte and frame rate.
const std::chrono::seconds ShmdataStat::kDefaultUpdateIntervalInSeconds = std::chrono::seconds(3);
const std::chrono::milliseconds ShmdataStat::kDefaultUpdateInterval =
    std::chrono::duration_cast<std::chrono::milliseconds>(
        ShmdataStat::kDefaultUpdateIntervalInSeconds);

void ShmdataStat::count_buffer(size_t buffer_size) {
  bytes_ += buffer_size;
  ++accesses_;
}

void ShmdataStat::reset() {
  bytes_ = 0;
  accesses_ = 0;
}

std::function<void(const ShmdataStat&)> ShmdataStat::make_tree_updater(Quiddity* quid,
                                                                       const std::string& key) {
  return [quid, key](const ShmdataStat& stat) {
    auto tree = InfoTree::make();
    tree->graft(".byte_rate",
                InfoTree::make(stat.bytes_ / ShmdataStat::kDefaultUpdateIntervalInSeconds.count()));
    tree->graft(
        ".rate",
        InfoTree::make(stat.accesses_ / ShmdataStat::kDefaultUpdateIntervalInSeconds.count()));
    quid->graft_tree(key + ".stat", tree);
  };
}

void ShmdataStat::update_tree(const InfoTree::ptr& tree, const std::string& key) const {
  tree->graft(key + ".stat.byte_rate",
              InfoTree::make(bytes_ / ShmdataStat::kDefaultUpdateIntervalInSeconds.count()));
  tree->graft(key + ".stat.rate",
              InfoTree::make(accesses_ / ShmdataStat::kDefaultUpdateIntervalInSeconds.count()));
}

}  // namespace
