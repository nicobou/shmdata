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

#include "./gst-initialized.hpp"

namespace switcher {
GstInitialized::GstInitialized() {
  if (!gst_is_initialized()) gst_init(nullptr, nullptr);
  registry_ = gst_registry_get();
  // TODO add option for scanning a path
  gst_registry_scan_path(registry_, "/usr/local/lib/gstreamer-1.0/");
  gst_registry_scan_path(registry_, "/usr/lib/gstreamer-1.0/");
}

bool GstInitialized::set_plugin_as_primary(const std::string& plugin_name, int priority) {
  GstPluginFeature* plugin = gst_registry_lookup_feature(registry_, plugin_name.c_str());
  if(plugin != nullptr) {
    gst_plugin_feature_set_rank(plugin, GST_RANK_PRIMARY + priority);
    gst_object_unref(plugin);
    return true;
  }
  gst_object_unref(plugin); 
  return false;
}
}  // namespace switcher
