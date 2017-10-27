/*
 * Copyright (C) 2015 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#ifndef _SILENT_LOGGER_H_
#define _SILENT_LOGGER_H_

#include <iostream>
#include "./base-logger.hpp"

namespace switcher {

class SilentLogger : public BaseLogger {
 private:
  void on_error(std::string&&) final {}
  void on_critical(std::string&&) final {}
  void on_warning(std::string&&) final {}
  void on_message(std::string&&) final {}
  void on_info(std::string&&) final {}
  void on_debug(std::string&&) final {}
};

}  // namespace shmdata
#endif
