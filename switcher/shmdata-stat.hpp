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

#ifndef __SWITCHER_SHMDATA_STAT_H__
#define __SWITCHER_SHMDATA_STAT_H__

#include <cstddef>
#include "./quiddity.hpp"

namespace switcher {
struct ShmdataStat {
  static const std::chrono::seconds kDefaultUpdateIntervalInSeconds;
  static const std::chrono::milliseconds kDefaultUpdateInterval;

  size_t bytes_{0};
  size_t accesses_{0};
  void count_buffer(size_t buffer_size);
  void reset();
  void update_tree(const InfoTree::ptr& tree, const std::string& key) const;
  static std::function<void(const ShmdataStat&)> make_tree_updater(Quiddity* quid,
                                                                   const std::string& key);
};

}  // namespace switcher
#endif
