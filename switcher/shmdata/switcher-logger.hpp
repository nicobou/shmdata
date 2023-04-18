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

#include "switcher/logger/logger.hpp"

namespace switcher {
namespace shmdata {

class SwitcherLogger : public ::shmdata::AbstractLogger {
 public:
  SwitcherLogger(logger::Logger* logger) : logger_(logger) {}
  SwitcherLogger() = delete;

 private:
  const logger::Logger* logger_;
  void on_error(std::string&& str) final { logger_->sw_error(str); }
  void on_critical(std::string&& str) final { logger_->sw_critical(str); }
  void on_warning(std::string&& str) final { logger_->sw_warning(str); }
  void on_message(std::string&& str) final { logger_->sw_trace(str); }
  void on_info(std::string&& str) final { logger_->sw_info(str); }
  void on_debug(std::string&& str) final { logger_->sw_debug(str); }
};

}  // namespace shmdata
}  // namespace switcher
#endif
