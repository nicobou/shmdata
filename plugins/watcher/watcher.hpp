/*
 * This file is part of switcher-watcher.
 *
 * switcher-watcher is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_WATCHER_H__
#define __SWITCHER_WATCHER_H__

#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include "switcher/quiddity/container.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/quiddity/startable.hpp"
#include "switcher/shmdata/follower.hpp"
#include "switcher/utils/periodic-task.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class Watcher : public Quiddity, public Startable {
 public:
  enum class DirectoryStatus { ERROR, ABSENT, PRESENT, IS_FILE };
  Watcher(quiddity::Config&&);
  ~Watcher();
  Watcher(const Watcher&) = delete;
  Watcher& operator=(const Watcher&) = delete;

 private:
  std::vector<std::tuple<std::string, std::unique_ptr<shmdata::Follower>>> followers_;
  std::unique_ptr<PeriodicTask<>> readEventsTask_;
  std::string directory_{"."};
  property::prop_id_t directory_id_;
  bool create_dir_{false};
  property::prop_id_t create_dir_id_;

  // Shmdata methods
  bool start() final;
  bool stop() final;

  void watch_events();
  void create_follower(const std::string& shmpath);
  DirectoryStatus dir_exists(const std::string path) const;
};
SWITCHER_DECLARE_PLUGIN(Watcher);
}  // namespace quiddities
}  // namespace switcher
#endif