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

#ifndef __SWITCHER_SWITCHER_LOGGER_LOG_H__
#define __SWITCHER_SWITCHER_LOGGER_LOG_H__

#include <filesystem>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <uuid/uuid.h>

#include "switcher/utils/safe-bool-idiom.hpp"
#include "switcher/configuration/configuration.hpp"

namespace fs = std::filesystem;

namespace switcher {
namespace logger {

class Log : public SafeBoolIdiom {
 public:
  Log() = delete;
  /**
   * @brief Initialize the log
   * @details initializes the logger and adds a `console sink` to it.
   *
   * @param logger_name A name for the logger. It must not be used by an other logger.
   * @param debug A boolean that indicates if a `debug` log level should be enforced
   * @return A shared pointer to the logger
   */
  Log(const std::string& logger_name, switcher::Configuration* conf, bool debug);

  /**
   * @brief Getter for the spdlogger
   * @return A shared pointer to the logger
   */
  std::shared_ptr<spdlog::logger> get_logger();

  /**
   * @brief set the log level
   *
   * @param level_name the log level name ()
   * @return the log level name that has been set
   */
  std::string set_log_level(const std::string& level_name);

 private:
  /**
   * @brief the file path of the rotating log file. Disabled until set_rotating_file
   * is invoked
   **/
  std::string filepath_{};

  /**
   * @brief Universally unique identifier for the log session
   * @details A universally unique identifier is a 128-bit label used for information in computer
   * systems.
   */
  const std::string uuid_;
  /**
   * @brief A shared pointer to a registered logger
   * @details When `making` the Switcher instance, a logger is initialized and outputs only to the
   * console until the instance is configured. Once the instance can check for `logs` settings, the
   * logger will also outputs to a log file on the system.
   */
  std::shared_ptr<spdlog::logger> logger_;
  /**
   * @brief Boolean value that forces at least debug level if true
   **/
  bool debug_;

  /**
   * @brief Boolean value telling if the Log instance has been correctly initialized. It is
   * used with the SafeBoolIdiom system.
   **/
  bool is_valid_{true};

  /**
   * @brief Generate a universally unique identifier (uuid)
   * @return A string that contains the generated uuid
   */
  std::string make_uuid() const;

  /**
   * @brief SafeBoolIdiom implementation
   * @detail tells if the Log instance has initialized correctly
   **/
  bool safe_bool_idiom() const final { return is_valid_; }

  /**
   * @brief generate the spdlogger pattern for the logs
   *
   * @param the logger name
   * @param the uuid identifier for this logging session
   * @return the string pattern in spdlogger format
   **/
  static std::string make_logger_pattern(const std::string& name, const std::string& uuid);

  /**
   * Gets the default path of the log file
   * @return The path of the log file
   */
  static fs::path get_default_log_path();

  /**
   * Maximum number of files for the rotating log.
   */
  static const size_t cMaxFiles = 100;
};
}  // namespace logger
}  // namespace switcher

#endif
