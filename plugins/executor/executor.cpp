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

#include "./executor.hpp"
#include <unistd.h>
#include "switcher/utils/scope-exit.hpp"

namespace switcher {
namespace quiddities {

static volatile sig_atomic_t child_dead = 0;
extern "C" void signal_cb(int signal_number) { child_dead = 1; }

SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Executor,
                                     "executor",
                                     "Command line launcher",
                                     "utils",
                                     "reader",
                                     "Plugin for launching command lines",
                                     "LGPL",
                                     "Francis Lecavalier");

Executor::Executor(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      Startable(this),
      shmcntr_(static_cast<Quiddity*>(this)),
      command_line_id_(
          pmanage<MPtr(&property::PBag::make_string)>("command_line",
                                                      [this](const std::string& val) {
                                                        command_line_ = val;
                                                        return true;
                                                      },
                                                      [this]() { return command_line_; },
                                                      "Command Line",
                                                      "Command line to execute",
                                                      command_line_)),
      autostart_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "autostart",
          [this](bool val) {
            autostart_ = val;
            return true;
          },
          [this]() { return autostart_; },
          "Autostart",
          "Execute command line on shmdata connect or not",
          autostart_)),
      restart_on_change_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "restart_on_change",
          [this](bool val) {
            restart_on_change_ = val;
            return true;
          },
          [this]() { return restart_on_change_; },
          "Restart on change",
          "Restart process execution when shmdata connections change or not",
          restart_on_change_)),
      periodic_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "periodic",
          [this](bool val) {
            periodic_ = val;
            return true;
          },
          [this]() { return periodic_; },
          "Make periodic",
          "Execute the command line again when the process finishes or not",
          periodic_)),

      // Whitelist caps in order to control connections with an Executor as a writer
      whitelist_caps_id_(
          pmanage<MPtr(&property::PBag::make_string)>("whitelist_caps",
                                                      [this](const std::string& val) {
                                                        whitelist_caps_ = val;
                                                        return true;
                                                      },
                                                      [this]() { return whitelist_caps_; },
                                                      "Whitelist compatible capabilities",
                                                      "Apply capabilities to executed command line",
                                                      whitelist_caps_)) {
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return on_shmdata_connect(shmpath); },
      [this](const std::string& shmpath) { return on_shmdata_disconnect(shmpath); },
      [this]() { return on_shmdata_disconnect_all(); },
      [this](const std::string& caps) { return can_sink_caps(caps); },
      std::numeric_limits<unsigned int>::max());
  memset(&sigchld_action_, 0, sizeof(sigchld_action_));
  sigchld_action_.sa_handler = signal_cb;
  sigaction(SIGCHLD, &sigchld_action_, nullptr);
  posix_spawnattr_init(&attr_);
  posix_spawnattr_setpgroup(&attr_, 0);
  posix_spawnattr_setflags(&attr_, POSIX_SPAWN_SETPGROUP);

  // Launch periodic checking
  monitoring_task_ = std::make_unique<PeriodicTask<>>([this]() { monitor_process(); },
                                                      std::chrono::milliseconds(500));
}

Executor::~Executor() {
  stop();
  monitor_process();
  posix_spawnattr_destroy(&attr_);
}

bool Executor::start() {
  if (child_pid_ != 0) {
    error("executor already started");
    return false;
  }
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

  // Pipes creation and setup
  if (pipe(cout_pipe_) || pipe(cerr_pipe_)) {
    error("error while setting up interprocess communication. Error: %", strerror(errno));
  }
  posix_spawn_file_actions_init(&act_);
  posix_spawn_file_actions_addclose(&act_, cout_pipe_[0]);
  posix_spawn_file_actions_addclose(&act_, cerr_pipe_[0]);
  posix_spawn_file_actions_adddup2(&act_, cout_pipe_[1], 1);
  posix_spawn_file_actions_adddup2(&act_, cerr_pipe_[1], 2);
  posix_spawn_file_actions_addclose(&act_, cout_pipe_[1]);
  posix_spawn_file_actions_addclose(&act_, cerr_pipe_[1]);

  // Process launching
  user_stopped_ = false;
  int status2 = posix_spawnp(&child_pid_, program.c_str(), &act_, &attr_, words.we_wordv, environ);
  if (status2 != 0) {
    error("process could not be executed. Error: %", strerror(status2));
    wordfree(&words);
    return false;
  }

  wordfree(&words);
  close(cout_pipe_[1]);
  close(cerr_pipe_[1]);

  return true;
}

bool Executor::stop() {
  if (child_pid_ != 0) {
    user_stopped_ = true;
    killpg(child_pid_, SIGTERM);
  }
  return true;
}

bool Executor::on_shmdata_connect(const std::string& shmpath) {
  if (stringutils::ends_with(shmpath, "video")) {
    if (!shmpath_video_.empty()) follower_video_.reset();
    shmpath_video_ = shmpath;
    follower_video_ = std::make_unique<shmdata::Follower>(this,
                                                          shmpath_video_,
                                                          nullptr,
                                                          nullptr,
                                                          nullptr,
                                                          shmdata::Stat::kDefaultUpdateInterval,
                                                          shmdata::Follower::Direction::reader,
                                                          true);
  } else if (stringutils::ends_with(shmpath, "audio")) {
    if (!shmpath_audio_.empty()) follower_audio_.reset();
    shmpath_audio_ = shmpath;
    follower_audio_ = std::make_unique<shmdata::Follower>(this,
                                                          shmpath_audio_,
                                                          nullptr,
                                                          nullptr,
                                                          nullptr,
                                                          shmdata::Stat::kDefaultUpdateInterval,
                                                          shmdata::Follower::Direction::reader,
                                                          true);
  } else {
    if (!shmpath_.empty()) follower_.reset();
    shmpath_ = shmpath;
    follower_ = std::make_unique<shmdata::Follower>(this,
                                                    shmpath_,
                                                    nullptr,
                                                    nullptr,
                                                    nullptr,
                                                    shmdata::Stat::kDefaultUpdateInterval,
                                                    shmdata::Follower::Direction::reader,
                                                    true);
  }
  if (child_pid_ != 0) {
    if (restart_on_change_) {
      if (!restart_) {
        stop();
        restart_ = true;
      }
    }
  } else {
    if (autostart_) {
      return start();
    }
  }
  return true;
}

bool Executor::on_shmdata_disconnect(const std::string& shmpath) {
  if (stringutils::ends_with(shmpath, "video")) {
    follower_video_.reset();
    shmpath_video_.clear();
  } else if (stringutils::ends_with(shmpath, "audio")) {
    follower_audio_.reset();
    shmpath_audio_.clear();
  } else {
    follower_.reset();
    shmpath_.clear();
  }
  if (child_pid_ != 0) {
    if (shmpath_video_.empty() && shmpath_audio_.empty() && shmpath_.empty()) {
      return stop();
    }
    if (restart_on_change_) {
      if (!restart_) {
        stop();
        restart_ = true;
      }
    }
  }
  return true;
}

bool Executor::on_shmdata_disconnect_all() {
  follower_video_.reset();
  follower_audio_.reset();
  follower_.reset();
  shmpath_video_.clear();
  shmpath_audio_.clear();
  shmpath_.clear();
  return stop();
}

bool Executor::can_sink_caps(const std::string& str_caps) {
  // by default, it accepts all caps if `whitelist_caps_` is empty
  if (whitelist_caps_.empty()) return true;

  GstCaps* caps = gst_caps_from_string(str_caps.c_str());

  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };

  GstStructure* caps_struct = gst_caps_get_structure(caps, 0);
  std::string caps_name(gst_structure_get_name(caps_struct));

  return (int) whitelist_caps_.find(caps_name, 0) >= 0;
}

void Executor::monitor_process() {
  if (child_pid_ > 0) {
    if (child_dead == 1) {
      pid_t pid = waitpid(child_pid_, &child_return_code_, WNOHANG);
      if (pid > 0 && pid == child_pid_) {
        child_dead = 0;
        child_pid_ = 0;
        clean_up_child_process();
        if ((periodic_ && !user_stopped_) || restart_) {
          start();
          restart_ = false;
        } else {
          pmanage<MPtr(&property::PBag::set_str_str)>("started", "false");
        }
      }
    }
  }
}

bool Executor::read_outputs() {
  std::string cout_buffer(1024, ' ');
  std::string cerr_buffer(1024, ' ');
  int cout_bytes_read = 0;
  int cerr_bytes_read = 0;

  pollfd plist[2] = {{cout_pipe_[0], POLLIN}, {cerr_pipe_[0], POLLIN}};

  if (poll(plist, 2, 10) > 0) {
    if (plist[0].revents & POLLIN) {
      cout_bytes_read = read(cout_pipe_[0], cout_buffer.data(), cout_buffer.length());
    } else if (plist[1].revents & POLLIN) {
      cerr_bytes_read = read(cerr_pipe_[0], cerr_buffer.data(), cerr_buffer.length());
    }
  }

  // update stdout value
  std::string escaped_stdout = switcher::stringutils::escape_json(
      cout_buffer.substr(0, static_cast<size_t>(cout_bytes_read)));

  bool is_updated = graft_output("stdout", escaped_stdout);

  // update stderr value
  std::string escaped_stderr = switcher::stringutils::escape_json(
      cerr_buffer.substr(0, static_cast<size_t>(cerr_bytes_read)));

  return graft_output("stderr", escaped_stderr) || is_updated;
}

bool Executor::graft_output(const std::string& type, const std::string& escaped_value) {
  bool has_changed = true;
  std::string path = ".output." + type + ".";
  auto branch = get_tree(path);

  if (branch->has_data()) {
    std::string old = branch->read_data();
    has_changed = old != escaped_value;
  }

  if (has_changed) {
    info("Grafting % in %", escaped_value, type);
    graft_tree(path, InfoTree::make(escaped_value));
  }

  return has_changed;
}

void Executor::clean_up_child_process() {
  // Check child's outputs
  bool is_updated = read_outputs();
  is_updated = graft_output("return_code", std::to_string(child_return_code_)) || is_updated;

  close(cout_pipe_[0]);
  close(cerr_pipe_[0]);
  posix_spawn_file_actions_destroy(&act_);

  if (is_updated) {
    info("'%' output has been updated.", command_line_);
  }
}
}  // namespace quiddities
}  // namespace switcher
