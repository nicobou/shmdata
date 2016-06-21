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

#ifndef __SWITCHER_GOBJECT_WRAPPER_H__
#define __SWITCHER_GOBJECT_WRAPPER_H__

#include <glib-object.h>
#include <map>
#include <memory>
#include <string>
// #include "./gobject-custom-property.hpp"
// #include "./gobject-custom-signal.hpp"

namespace switcher {
struct _MyObject;
struct _MyObjectClass;

class GObjectWrapper {
 public:
  typedef std::shared_ptr<GObjectWrapper> ptr;
  GObjectWrapper();
  ~GObjectWrapper();
  GObjectWrapper(const GObjectWrapper&) = delete;
  GObjectWrapper& operator=(const GObjectWrapper&) = delete;

  GObject* get_gobject();

  // signal
  static guint make_signal(GType return_type, guint n_params, GType* param_types);

 private:
  struct _MyObject* my_object_;

  // ---------- signals
  // static std::map<guint, GObjectCustomSignal::ptr> custom_signals_;
  static guint next_signal_num_;  // this is only for generation of unique signal names
  std::map<std::string, void*> signal_user_datas_;
};

}  // namespace switcher
#endif
