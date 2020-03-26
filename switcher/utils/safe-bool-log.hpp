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

#ifndef __SWITCHER_SAFE_BOOL_LOG_H__
#define __SWITCHER_SAFE_BOOL_LOG_H__

#include <string>
#include "./safe-bool-idiom.hpp"

namespace switcher {

class SafeBoolLog : public SafeBoolIdiom {
 public:
  std::string msg() const { return msg_; };

 protected:
  void set_safe_bool_log_msg(const std::string& msg) { msg_ = msg; }

 private:
  std::string msg_;
};

}  // namespace switcher

#endif
