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

#ifndef __SWITCHER_SWITCHER_LOGGER_LOGGABLE_H__
#define __SWITCHER_SWITCHER_LOGGER_LOGGABLE_H__

#include <spdlog/spdlog.h>

#include <string>

#include "switcher/configuration/configuration.hpp"
#include "switcher/logger/log.hpp"

namespace switcher {
namespace logger {

/**
 * A wrapper class to be inherited by other classes (like Switcher and Quiddity)
 * in order to enable logging methods.
 *
 * The logging methods are employing the {fmt} library https://fmt.dev/latest/index.html
 **/
class Logger {
 public:
  Logger() = delete;
  Logger(const std::string& switcher_name, switcher::Configuration* conf, bool debug);
  Logger(const Logger& obj);

  template <typename FormatString, typename... Args>
  void sw_trace(const FormatString& fmt, Args&&... args) const {
    logger_->trace(std::forward<const FormatString&>(fmt), std::forward<Args>(args)...);
  }

  template <typename FormatString, typename... Args>
  void sw_info(const FormatString& fmt, Args&&... args) const {
    logger_->info(std::forward<const FormatString&>(fmt), std::forward<Args>(args)...);
  }

  template <typename FormatString, typename... Args>
  void sw_debug(const FormatString& fmt, Args&&... args) const {
    logger_->debug(std::forward<const FormatString&>(fmt), std::forward<Args>(args)...);
  }

  template <typename FormatString, typename... Args>
  void sw_warning(const FormatString& fmt, Args&&... args) const {
    logger_->warn(std::forward<const FormatString&>(fmt), std::forward<Args>(args)...);
  }

  template <typename FormatString, typename... Args>
  void sw_error(const FormatString& fmt, Args&&... args) const {
    logger_->error(std::forward<const FormatString&>(fmt), std::forward<Args>(args)...);
  }

  template <typename FormatString, typename... Args>
  void sw_critical(const FormatString& fmt, Args&&... args) const {
    logger_->critical(std::forward<const FormatString&>(fmt), std::forward<Args>(args)...);
  }

 private:
  /**
   * @brief The log life and configuration manager
   **/
  std::unique_ptr<Log> log_{};

  /**
   * @brief A shared pointer to a registered logger
   */
  mutable std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace logger
}  // namespace switcher

#endif
