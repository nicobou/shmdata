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

#include "./log.hpp"

#include <unistd.h>  // use of getgid()
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

namespace switcher {
namespace logger {

std::string Log::make_logger_pattern(const std::string& name, const std::string& uuid) {
  return "%Y-%m-%d %H:%M:%S.%e|" + name + "|" + uuid + "|%P|%t|%^%l%$: %v|%s:%#";
}

std::string Log::make_uuid() const {
  // declare structure to hold uuid in binary representation
  uuid_t binary_uuid;
  // generate a random binary uuid
  uuid_generate_random(binary_uuid);
  // declare variable to hold uuid in characters representation
  char uuid_chars[36];
  // convert uuid binary to character representation
  uuid_unparse_lower(binary_uuid, uuid_chars);
  // cast char array to string
  return std::string(uuid_chars);
}

    Log::Log(const std::string& logger_name, switcher::Configuration* conf, bool debug)
      : filepath_(conf->get_value(".logs.filepath")),
	uuid_(make_uuid()),
	logger_(spdlog::get(logger_name)),
	debug_(debug) {
      if (logger_) {
        logger_->warn("A logger named {} already exists, reusing existing session", logger_name);
        is_valid_ = false;
	return;
      }
    
      // init console sink
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    
      // init rotating file sink
      if (filepath_.empty()) {
	filepath_ = std::string(get_default_log_path());
      }
      // configure log file path
      conf->set_value(".logs.filepath", filepath_);

      size_t max_files = conf->get_value(".logs.max_files");
      if (!max_files) {
	max_files = 3;
      }
      if (max_files > cMaxFiles) {
        max_files = cMaxFiles;
      }
      conf->set_value(".logs.max_files", max_files);
    
      size_t max_size = conf->get_value(".logs.max_size");
      if (!max_size) {
	max_size = 1048576 * 100;  // 100MB
      } else {
	max_size = 1048576 * max_size;
      }
      conf->set_value(".logs.max_size", max_size);
    
      auto file_sink =
	std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filepath_, max_size, max_files);
    
      // sinks init list to construct logger
      spdlog::sinks_init_list sinks{console_sink, file_sink};
    
      // init logger from sinks
      logger_ = std::make_shared<spdlog::logger>(logger_name, sinks);
    
      // compute pattern string
      logger_->set_pattern(make_logger_pattern(logger_name, uuid_));
    
      // register logger so that it can be accessed globally
      spdlog::register_logger(logger_);
    
      // set log level
      const std::string level_name = conf->get_value(".logs.log_level");
      conf->set_value(".logs.log_level", set_log_level(level_name));
    }
  
    std::string Log::set_log_level(const std::string& level_name) {
      auto level = spdlog::level::from_str(level_name);

      // debug takes priority over configured log level
      if (debug_ && level > spdlog::level::debug) {
        logger_->debug("Debug level forced for the logger named {}", logger_->name());
        logger_->set_level(spdlog::level::debug);
	return "debug";
      } else {
	logger_->set_level(level);
      }
      return level_name;
    }

    std::shared_ptr<spdlog::logger> Log::get_logger() { return logger_; }

    fs::path Log::get_default_log_path() {
      if (getgid()) {
	const auto xsh_env = std::getenv("XDG_STATE_HOME");
	const auto xsh_default_path = fs::path(std::getenv("HOME")) / ".local" / "state";
	const auto xsh_dir = fs::path(xsh_env ? std::string(xsh_env) : std::string(xsh_default_path));
	return xsh_dir / "switcher" / "logs" / "switcher.log";
      } else {  // running as root, use default log file path
	return "/var/log/switcher/switcher.log";
      }
    }

    }  // namespace logger
    }  // namespace switcher
