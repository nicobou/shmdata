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
class StartableQuiddity {
 public:
  StartableQuiddity() = default;
  virtual ~StartableQuiddity() = default;
  StartableQuiddity(const StartableQuiddity&) = delete;
  StartableQuiddity& operator=(const StartableQuiddity&) = delete;

  static const std::string disabledWhenStartedMsg;
  static const std::string disabledWhenStopedMsg;

 protected:
  void init_startable(void* quiddity);  // FIXME find a way to avoid invoking init_startable (this)
                                        // in quiddities (policies)
  bool is_started() const;

 private:
  virtual bool start() = 0;
  virtual bool stop() = 0;
  bool __started_{false};
};
}  // namespace switcher
#endif
