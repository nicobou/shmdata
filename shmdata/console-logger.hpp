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

#ifndef _CONSOLE_LOGGER_H_
#define _CONSOLE_LOGGER_H_

#include <iostream>
#include "./abstract-logger.hpp"

namespace shmdata {

class ConsoleLogger : public AbstractLogger {
 public:
  void set_debug(bool debug) { debug_ = debug; }

 private:
  bool debug_{true};
  void on_error(std::string&& str) final {
    std::cerr << "\033[1;31merror: " << str << "\033[0m" << std::endl;
  }
  void on_critical(std::string&& str) final {
    std::cerr << "\033[1;31mcritical: " << str << "\033[0m" << std::endl;
  }
  void on_warning(std::string&& str) final {
    std::cerr << "\033[1;33mwarning: " << str << "\033[0m" << std::endl;
  }
  void on_message(std::string&& str) final { std::cout << "message: " << str << std::endl; }
  void on_info(std::string&& str) final { std::cout << "info: " << str << std::endl; }
  void on_debug(std::string&& str) final {
    if (debug_) std::cout << "\033[0;33mdebug: " << str << "\033[0m" << std::endl;
  }
};

}  // namespace shmdata
#endif
