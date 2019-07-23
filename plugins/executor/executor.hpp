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

#ifndef __SWITCHER_EXECUTOR_H__
#define __SWITCHER_EXECUTOR_H__

#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <wordexp.h>
#include <memory>
#include <regex>
#include <string>
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {

std::function<void(int)> cb_wrapper;

class Executor : public Quiddity, public StartableQuiddity {
 public:
  Executor(quid::Config&&);
  ~Executor();
  Executor(const Executor&) = delete;
  Executor& operator=(const Executor&) = delete;

 private:
  static void clean_up_child_process_static(int signal_number);

  bool start() final;
  bool stop() final;
  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect(const std::string& shmpath);
  bool on_shmdata_disconnect_all();
  bool can_sink_caps(std::string str_caps);
  pid_t spawn_child(char* program, char** arg_list);
  void clean_up_child_process(int signal_number);
  void read_outputs();

  struct sigaction sigchld_action_;
  int cout_pipe_[2];
  int cerr_pipe_[2];
  bool user_stopped_{false};
  pid_t child_pid_;
  posix_spawnattr_t attr_;
  posix_spawn_file_actions_t act_;
  ShmdataConnector shmcntr_;
  std::string shmpath_{};
  std::string shmpath_audio_{};
  std::string shmpath_video_{};
  std::unique_ptr<ShmdataFollower> follower_video_{nullptr};
  std::unique_ptr<ShmdataFollower> follower_audio_{nullptr};
  std::unique_ptr<ShmdataFollower> follower_{nullptr};

  std::string command_line_{};
  PContainer::prop_id_t command_line_id_;
  bool autostart_{false};
  PContainer::prop_id_t autostart_id_;
  bool periodic_{false};
  PContainer::prop_id_t periodic_id_;
};
SWITCHER_DECLARE_PLUGIN(Executor);
}  // namespace switcher
#endif