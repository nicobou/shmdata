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

#ifndef __SWITCHER_SHMDATA_SWITCHER_LOGGER_H__
#define __SWITCHER_SHMDATA_SWITCHER_LOGGER_H__

#include <shmdata/abstract-logger.hpp>

namespace switcher {
namespace shmdata {

class SwitcherLogger : public ::shmdata::AbstractLogger {
 public:
  std::shared_ptr<spdlog::logger> logger;
  SwitcherLogger() : logger(spdlog::get("switcher")) {}

 private:
  void on_error(std::string&& str) final { LOGGER_ERROR(this->logger, str); }
  void on_critical(std::string&& str) final { LOGGER_CRITICAL(this->logger, str); }
  void on_warning(std::string&& str) final { LOGGER_WARN(this->logger, str); }
  void on_message(std::string&& str) final { LOGGER_INFO(this->logger, str); }
  void on_info(std::string&& str) final { LOGGER_INFO(this->logger, str); }
  void on_debug(std::string&& str) final { LOGGER_DEBUG(this->logger, str); }
};

}  // namespace shmdata
}  // namespace switcher
#endif
