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

#include <glib.h>
#include <memory>
#include "./custom-property-helper.hpp"

namespace switcher {
class StartableQuiddity {
 public:
  typedef std::shared_ptr<StartableQuiddity> ptr;
  StartableQuiddity();
  virtual ~StartableQuiddity();
  StartableQuiddity(const StartableQuiddity &) = delete;
  StartableQuiddity &operator=(const StartableQuiddity &) = delete;

 protected:
  void init_startable(void *quiddity);
  bool is_started() const;
  // find a way to avoid invoking init_startable (this) in quiddities (policies)

 private:
  virtual bool start() = 0;
  virtual bool stop() = 0;
  static gboolean get_started(void *user_data);
  static void set_started(gboolean started, void *user_data);

  bool started_{false};
  GParamSpec *started_prop_{nullptr};  // FIXME should be static
  CustomPropertyHelper::ptr startable_custom_props_;
};
}  // namespace switcher
#endif
