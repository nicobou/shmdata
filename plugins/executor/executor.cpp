/*
 * This file is part of switcher-executor.
 *
 * switcher-executor is free software; you can redistribute it and/or
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

#include "executor.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Executor,
                                     "executor",
                                     "Command line launcher",
                                     "utils",
                                     "reader",
                                     "Plugin for launching command lines",
                                     "LGPL",
                                     "Francis Lecavalier");

Executor::Executor(quid::Config&& conf)
    : Quiddity(std::forward<quid::Config>(conf)),
      shmcntr_(static_cast<Quiddity*>(this)),
      command_line_id_(
          pmanage<MPtr(&PContainer::make_string)>("command_line",
                                                  [this](const std::string& val) {
                                                    command_line_ = val;
                                                    return true;
                                                  },
                                                  [this]() { return command_line_; },
                                                  "Command Line",
                                                  "Command line to execute",
                                                  command_line_)),
      autostart_id_(pmanage<MPtr(&PContainer::make_bool)>("autostart",
                                                          [this](bool val) {
                                                            autostart_ = val;
                                                            return true;
                                                          },
                                                          [this]() { return autostart_; },
                                                          "Autostart",
                                                          "Execute command line on shmdata connect or not",
                                                          autostart_)) {
  init_startable(this);
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return on_shmdata_connect(shmpath); },
      [this](const std::string& shmpath) { return on_shmdata_disconnect(shmpath); },
      [this]() { return on_shmdata_disconnect_all(); },
      [this](const std::string& caps) { return can_sink_caps(caps); },
      std::numeric_limits<unsigned int>::max());
  memset(&sigchld_action_, 0, sizeof(sigchld_action_));
  sigchld_action_.sa_handler = &Executor::clean_up_child_process;
  sigaction(SIGCHLD, &sigchld_action_, nullptr);
  posix_spawnattr_init(&attr_);
  posix_spawnattr_setpgroup(&attr_, 0);
  posix_spawnattr_setflags(&attr_, POSIX_SPAWN_SETPGROUP);
}

Executor::~Executor() {
  stop();
  posix_spawnattr_destroy(&attr_);
}

bool Executor::start() {
  stop();
  if (command_line_.empty()) {
    error("command_line must not be empty");
    return false;
  }

  std::string program = command_line_.substr(0, command_line_.find(' '));
  std::string args = command_line_;
  // Replace "_shmpath_*_" substrings with connected shmpaths, if they exist.
  if (!shmpath_audio_.empty()) {
    args = std::regex_replace(args, std::regex("_shmpath_audio_"), shmpath_audio_);
  }
  if (!shmpath_video_.empty()) {
    args = std::regex_replace(args, std::regex("_shmpath_video_"), shmpath_video_);
  }
  if (!shmpath_.empty()) {
    args = std::regex_replace(args, std::regex("_shmpath_"), shmpath_);
  }

  // Arguments parsing
  wordexp_t words;
  int status1 = wordexp(args.c_str(), &words, 0);
  if (status1 != 0) {
    error("error while parsing command line. Error: %", strerror(status1));
    wordfree(&words);
    return false;
  }

  // Process launching
  int status2 = posix_spawn(&child_pid_, program.c_str(), nullptr, &attr_, words.we_wordv, environ);
  if (status2 != 0) {
    warning("process could not be executed. Error: %", strerror(status2));
    wordfree(&words);
    return false;
  }
  info("'%' was succesfully executed.", args);
  wordfree(&words);
  return true;
}

bool Executor::stop() {
  if (child_pid_ != 0) {
    killpg(child_pid_, SIGTERM);
    child_pid_ = 0;
    return true;
  }
  return false;
}

bool Executor::on_shmdata_connect(const std::string& shmpath) {
  if (StringUtils::ends_with(shmpath, "video")) {
    if (!shmpath_video_.empty()) follower_video_.reset();
    shmpath_video_ = shmpath;
    follower_video_ = std::make_unique<ShmdataFollower>(this,
                                                        shmpath_video_,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        ShmdataStat::kDefaultUpdateInterval,
                                                        ShmdataFollower::Direction::reader,
                                                        true);
  } else if (StringUtils::ends_with(shmpath, "audio")) {
    if (!shmpath_audio_.empty()) follower_audio_.reset();
    shmpath_audio_ = shmpath;
    follower_audio_ = std::make_unique<ShmdataFollower>(this,
                                                        shmpath_audio_,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        ShmdataStat::kDefaultUpdateInterval,
                                                        ShmdataFollower::Direction::reader,
                                                        true);
  } else {
    if (!shmpath_.empty()) follower_.reset();
    shmpath_ = shmpath;
    follower_ = std::make_unique<ShmdataFollower>(this,
                                                  shmpath_,
                                                  nullptr,
                                                  nullptr,
                                                  nullptr,
                                                  ShmdataStat::kDefaultUpdateInterval,
                                                  ShmdataFollower::Direction::reader,
                                                  true);
  }
  if (autostart_) return start();
  return true;
}

bool Executor::on_shmdata_disconnect(const std::string& shmpath) {
  if (StringUtils::ends_with(shmpath, "video")) {
    follower_video_.reset();
    shmpath_video_.clear();
  } else if (StringUtils::ends_with(shmpath, "audio")) {
    follower_audio_.reset();
    shmpath_audio_.clear();
  } else {
    follower_.reset();
    shmpath_.clear();
  }
  if (autostart_) return stop();
  return true;
}

bool Executor::on_shmdata_disconnect_all() {
  follower_video_.reset();
  follower_audio_.reset();
  follower_.reset();
  shmpath_video_.clear();
  shmpath_audio_.clear();
  shmpath_.clear();
  if (autostart_) return stop();
  return true;
}

bool Executor::can_sink_caps(std::string /*str_caps*/) { return true; }

void Executor::clean_up_child_process(int /*signal_number*/) {
  wait(nullptr);
}
}  // namespace switcher
