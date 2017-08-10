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

#include "./custom-save-plugin.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(CustomSavePlugin,
                                     "custom-save",
                                     "Custom Save Plugin",
                                     "test",
                                     "",
                                     "Custom Save plugin for testing/example purpose",
                                     "LGPL",
                                     "Nicolas Bouillot");

CustomSavePlugin::CustomSavePlugin(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      has_loaded_custom_state_id_(pmanage<MPtr(&PContainer::make_bool)>(
          "has_loaded_custom_state",
          nullptr,
          [this]() { return has_loaded_custom_state_ && on_loaded_called_; },
          "Has loaded",
          "A custom state has been loaded with success",
          has_loaded_custom_state_ && on_loaded_called_)),
      has_saved_custom_state_id_(pmanage<MPtr(&PContainer::make_bool)>(
          "has_saved_custom_state",
          nullptr,
          [this]() { return on_saving_called_ && on_saved_called_; },
          "Has Saved",
          "A custom state has been saved with success",
          has_loaded_custom_state_)) {}


InfoTree::ptr CustomSavePlugin::on_saving() {
  InfoTree::ptr precious_information = InfoTree::make();
  precious_information->graft(".val", InfoTree::make(val_));
  precious_information->graft(".is_something", InfoTree::make(is_something_));
  on_saving_called_ = true;
  return precious_information;
}

void CustomSavePlugin::on_saved() { on_saved_called_ = true; }

void CustomSavePlugin::on_loading(InfoTree::ptr&& tree) {
  if (tree->empty()) return;
  val_ = tree->branch_get_value(".val");
  is_something_ = tree->branch_get_value(".is_something");
  has_loaded_custom_state_ = true;
}

void CustomSavePlugin::on_loaded() { on_loaded_called_ = true; }

}  // namespace switcher
