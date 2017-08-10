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

#ifndef __SWITCHER_CUSTOM_SAVE_PLUGIN_H__
#define __SWITCHER_CUSTOM_SAVE_PLUGIN_H__

#include <memory>
#include <string>

#include "switcher/quiddity.hpp"

namespace switcher {
class CustomSavePlugin : public Quiddity {
 public:
  CustomSavePlugin(QuiddityConfiguration&&);
  ~CustomSavePlugin() = default;
  CustomSavePlugin(const CustomSavePlugin&) = delete;
  CustomSavePlugin& operator=(const CustomSavePlugin&) = delete;

 private:
  int val_{1};
  bool is_something_{true};
  // property used in order to implement the test
  bool has_loaded_custom_state_{false};
  bool on_loaded_called_{false};
  PContainer::prop_id_t has_loaded_custom_state_id_;
  bool on_saving_called_{false};
  bool on_saved_called_{false};
  PContainer::prop_id_t has_saved_custom_state_id_;
  InfoTree::ptr on_saving() final;
  void on_saved() final;
  void on_loading(InfoTree::ptr&& tree) final;
  void on_loaded() final;
};

SWITCHER_DECLARE_PLUGIN(CustomSavePlugin);

}  // namespace switcher
#endif
