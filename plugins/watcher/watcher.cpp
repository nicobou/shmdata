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

#include "watcher.hpp"

namespace fs = std::filesystem;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Watcher,
                                     "watcher",
                                     "Directory watcher",
                                     "utils",
                                     "writer",
                                     "Watch a directory for shmdatas",
                                     "LGPL",
                                     "Francis Lecavalier");

Watcher::Watcher(quid::Config&& conf)
    : Quiddity(std::forward<quid::Config>(conf)),
      StartableQuiddity(this),
      directory_id_(pmanage<MPtr(&PContainer::make_string)>("directory",
                                                            [this](const std::string& val) {
                                                              directory_ = val;
                                                              return true;
                                                            },
                                                            [this]() { return directory_; },
                                                            "Directory Path",
                                                            "Full path of the directory to watch",
                                                            directory_)),
      create_dir_id_(
          pmanage<MPtr(&PContainer::make_bool)>("create_directory",
                                                [this](bool val) {
                                                  create_dir_ = val;
                                                  return true;
                                                },
                                                [this]() { return create_dir_; },
                                                "Create directory",
                                                "Create the watched directory if it doesn't exist",
                                                create_dir_)) {}

Watcher::~Watcher() { stop(); }

bool Watcher::start() {
  stop();
  // Arguments validation
  if (directory_.empty()) {
    error("Directory path must not be empty");
    return false;
  }

  // Directory checking and creation
  DirectoryStatus status = dir_exists(directory_);
  if (status == DirectoryStatus::ERROR) {
    return false;
  }
  if (status == DirectoryStatus::IS_FILE) {
    error("% is a file, not a directory.", directory_);
    return false;
  }
  if (status == DirectoryStatus::ABSENT) {
    if (!create_dir_) {
      error("Directory % does not exist.", directory_);
      return false;
    }
    std::error_code ec;
    if (!fs::create_directories(directory_, ec)) {
      error("Directory could not be created. Error: %", ec.message());
      return false;
    }
    fs::permissions(directory_, fs::perms::all, fs::perm_options::replace);
  }

  // Launch periodic checking
  readEventsTask_ = std::make_unique<PeriodicTask<>>([this]() { watch_events(); },
                                                     std::chrono::milliseconds(500));

  return true;
}

bool Watcher::stop() {
  readEventsTask_ = nullptr;
  followers_.clear();
  return true;
}

void Watcher::watch_events() {
  // Check if a file was deleted
  auto it = followers_.begin();
  while (it != followers_.end()) {
    if (!fs::exists(fs::path(std::get<0>(*it)))) {
      it = followers_.erase(it);
    } else {
      ++it;
    }
  }

  // Check if a file was created
  for (auto& file : fs::recursive_directory_iterator(directory_)) {
    std::string path = file.path().string();
    if (std::find_if(followers_.begin(), followers_.end(), [path](const std::tuple<std::string, std::unique_ptr<ShmdataFollower>>& t) {return std::get<0>(t) == path;}) == followers_.end()) {
      if (fs::is_socket(file.path())) {
        create_follower(path);
      }
    }
  }
  return;
}

void Watcher::create_follower(const std::string& shmpath) {
  followers_.push_back(
      std::make_tuple(shmpath,
                      std::make_unique<ShmdataFollower>(this,
                                                        shmpath,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        ShmdataStat::kDefaultUpdateInterval,
                                                        ShmdataFollower::Direction::writer,
                                                        true)));
}

Watcher::DirectoryStatus Watcher::dir_exists(const std::string path) const {
  try {
    if (!fs::exists(path)) {
      return DirectoryStatus::ABSENT;
    }
    if (fs::is_directory(path)) {
      return DirectoryStatus::PRESENT;
    }
  } catch (const fs::filesystem_error& e) {
    error("Error while checking for directory. Error: %", e.what());
    return DirectoryStatus::ERROR;
  }
  return DirectoryStatus::IS_FILE;
}
}  // namespace switcher
