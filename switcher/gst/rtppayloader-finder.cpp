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

#include "./rtppayloader-finder.hpp"
#include "../utils/scope-exit.hpp"

namespace switcher {
namespace gst {
namespace rtppayloaderfinder {

// function used as a filter for selecting the appropriate rtp payloader
gboolean sink_factory_filter(GstPluginFeature* feature, gpointer data) {
  GstCaps* caps = (GstCaps*)data;
  // searching element factories only
  if (!GST_IS_ELEMENT_FACTORY(feature)) return FALSE;
  const gchar* klass = gst_element_factory_get_klass(GST_ELEMENT_FACTORY(feature));
  if (!(g_strrstr(klass, "Payloader") && g_strrstr(klass, "RTP"))) return FALSE;
  if (!gst_element_factory_can_sink_all_caps(GST_ELEMENT_FACTORY(feature), caps)) return FALSE;
  return TRUE;
}

// sorting factory by rank
gint sink_compare_ranks(GstPluginFeature* f1, GstPluginFeature* f2) {
  gint diff = gst_plugin_feature_get_rank(f2) - gst_plugin_feature_get_rank(f1);
  if (diff != 0) return diff;
  return g_strcmp0(gst_plugin_feature_get_name(f2), gst_plugin_feature_get_name(f1));
}

GstElementFactory* get_factory(const std::string& caps_str) {
  GstCaps* caps = gst_caps_from_string(caps_str.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };
  return get_factory_by_caps(caps);
}

GstElementFactory* get_factory_by_caps(GstCaps* caps) {
  if (nullptr == caps) return nullptr;
  GList* list = gst_registry_feature_filter(
      gst_registry_get(), (GstPluginFeatureFilter)sink_factory_filter, FALSE, caps);
  On_scope_exit { gst_plugin_feature_list_free(list); };
  list = g_list_sort(list, (GCompareFunc)sink_compare_ranks);
  // bypassing jpeg for high dimensions
  bool jpeg_payloader = true;
  GstStructure* caps_structure = gst_caps_get_structure(caps, 0);
  // check jpeg dimension are suported by jpeg payloader
  if (g_str_has_prefix(gst_structure_get_name(caps_structure), "image/jpeg")) {
    jpeg_payloader = false;
    // gint width = 0, height = 0;
    // if (gst_structure_get_int(caps_structure, "height", &height)) {
    //   if (height <= 0 || height > 2040)
    //     jpeg_payloader = false;
    // }
    // if (gst_structure_get_int(caps_structure, "width", &width)) {
    //   if (width <= 0 || width > 2040)
    //     jpeg_payloader = false;
    // }
  }
  if (list != nullptr && jpeg_payloader) return GST_ELEMENT_FACTORY(list->data);
  return nullptr;
}

}  // namespace rtppayloaderfinder
}  // namespace gst
}  // namespace switcher
