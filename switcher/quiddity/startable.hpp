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

#ifndef __SWITCHER_STARTABLE_QUIDDITY_H__
#define __SWITCHER_STARTABLE_QUIDDITY_H__

#include <string>

namespace switcher {
namespace quiddity {
namespace bundle {
class Bundle;
}  // namespace bundle

class Startable {
  friend bundle::Bundle;

 public:
  Startable(void* quiddity);
  virtual ~Startable() = default;
  Startable(const Startable&) = delete;
  Startable& operator=(const Startable&) = delete;

  static const std::string disabledWhenStartedMsg;
  static const std::string disabledWhenStopedMsg;

 protected:
  bool is_started() const;

 private:
  // default ctor is private in order to let the bundle class chose optionnaly if it is itself
  // startable or not
  Startable() = default;
  bool __started_{false};

  void init_startable(void* quiddity);
  virtual bool start() = 0;
  virtual bool stop() = 0;
};
}  // namespace quiddity
}  // namespace switcher
#endif
