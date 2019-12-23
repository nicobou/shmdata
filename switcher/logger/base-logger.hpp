/*
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

#ifndef _BASE_LOGGER_H_
#define _BASE_LOGGER_H_

#include <iostream>
#include <string>

#define MakeSwitcherLogLevel(NAME)                                                        \
 public:                                                                                  \
  template <typename... Targs>                                                            \
  void NAME(const char* format, const ::std::string& value, Targs... Fargs) {             \
    on_##NAME(make_string(                                                                \
        format, std::forward<const std::string&>(value), std::forward<Targs>(Fargs)...)); \
  }                                                                                       \
  void NAME(const char* format) { on_##NAME(make_string(format)); }                       \
                                                                                          \
 private:                                                                                 \
  virtual void on_##NAME(std::string&&){};

namespace switcher {

class BaseLogger {
 public:
  virtual ~BaseLogger() = default;
  MakeSwitcherLogLevel(error);
  MakeSwitcherLogLevel(critical);
  MakeSwitcherLogLevel(warning);
  MakeSwitcherLogLevel(message);
  MakeSwitcherLogLevel(info);
  MakeSwitcherLogLevel(debug);

 private:
  std::string make_string(const char* format) { return std::string(format); }
  template <typename... Targs>
  std::string make_string(const char* format, const std::string& value, Targs... Fargs) {
    std::string res;
    for (; *format != '\0'; format++) {
      if (*format == '%') {
        res.append(value);
        return res.append(make_string(format + 1, Fargs...));
      }
      res.append(format, 1);
    }
    return res;
  }
};

}  // namespace shmdata
#endif
