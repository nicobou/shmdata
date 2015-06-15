/*
 * This file is part of switcher-myplugin.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_DUMMY_PLUGIN_H__
#define __SWITCHER_DUMMY_PLUGIN_H__

#include <memory>
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"

namespace switcher {
class DummyPlugin: public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(DummyPlugin);
  DummyPlugin(const std::string &);
  ~DummyPlugin() = default;
  DummyPlugin(const DummyPlugin &) = delete;
  DummyPlugin &operator=(const DummyPlugin &) = delete;

 private:
  CustomPropertyHelper::ptr custom_props_;
  bool myprop_{false};
  GParamSpec *myprop_prop_{nullptr};
  std::string hello_{};

  bool init() final;
  bool start() final;
  bool stop() final;

  static gboolean get_myprop(void *user_data);
  static void set_myprop(gboolean myprop, void *user_data);
  static gchar *my_hello_world_method(gchar *first_arg, void *user_data);
};

SWITCHER_DECLARE_PLUGIN(DummyPlugin);

}  // namespace switcher
#endif
