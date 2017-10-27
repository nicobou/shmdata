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

#include "./base-logger.hpp"
#include "shmdata/abstract-logger.hpp"

namespace switcher {

class ShmdataSwitcherLogger : public shmdata::AbstractLogger {
 public:
  ShmdataSwitcherLogger() = delete;
  ShmdataSwitcherLogger(BaseLogger* log) : log_(log) {}

 private:
  void on_error(std::string&& str) final { log_->warning("ERROR: %", str); }
  void on_critical(std::string&& str) final { log_->critical("%", str); }
  void on_warning(std::string&& str) final { log_->warning("%", str); }
  void on_message(std::string&& str) final { log_->message("%", str); }
  void on_info(std::string&& str) final { log_->info("%", str); }
  void on_debug(std::string&& str) final { log_->debug("%", str); }

  BaseLogger* log_;
};

}  // namespace switcher
#endif
