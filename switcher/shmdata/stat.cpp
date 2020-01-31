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

#include "./stat.hpp"

namespace switcher {
namespace shmdata {

// We use two for simplicity here, tasks need to be created with milliseconds and it is better for
// precision to have seconds for the display of the byte and frame rate.
const std::chrono::milliseconds Stat::kDefaultUpdateInterval =
    std::chrono::milliseconds(3000);

void Stat::count_buffer(size_t buffer_size) {
  bytes_ += buffer_size;
  ++accesses_;
}

void Stat::reset() {
  bytes_ = 0;
  accesses_ = 0;
}

std::function<void(const Stat&)> Stat::make_tree_updater(quiddity::Quiddity* quid,
                                                                       const std::string& key,
                                                                       bool do_signal) {
  return [quid, key, do_signal](const Stat& stat) {
    auto tree = InfoTree::make();
    tree->graft(
        ".byte_rate",
        InfoTree::make(stat.bytes_ /
                       std::chrono::duration<float>(Stat::kDefaultUpdateInterval).count()));
    tree->graft(
        ".rate",
        InfoTree::make(stat.accesses_ /
                       std::chrono::duration<float>(Stat::kDefaultUpdateInterval).count()));
    quid->graft_tree(key + ".stat", tree, do_signal);
  };
}

void Stat::update_tree(const InfoTree::ptr& tree, const std::string& key) const {
  tree->graft(
      key + ".stat.byte_rate",
      InfoTree::make(bytes_ /
                     std::chrono::duration<float>(Stat::kDefaultUpdateInterval).count()));
  tree->graft(
      key + ".stat.rate",
      InfoTree::make(accesses_ /
                     std::chrono::duration<float>(Stat::kDefaultUpdateInterval).count()));
}

}  // namespace shmdata
}  // namespace switcher
