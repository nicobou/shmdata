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

#ifndef __SWITCHER_BOOL_LOG_H__
#define __SWITCHER_BOOL_LOG_H__

#include <string>

namespace switcher {
class BoolLog {
 public:
  BoolLog() = delete;
  BoolLog(bool is_valid) : is_valid_(is_valid), msg_() {}
  BoolLog(bool is_valid, const std::string& msg) : is_valid_(is_valid), msg_(msg) {}
  operator bool() const { return is_valid_; };
  std::string msg() const { return msg_; }

 private:
  bool is_valid_;
  std::string msg_;
};
}  // namespace switcher

#endif
