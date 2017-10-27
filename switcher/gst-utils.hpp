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

#ifndef __SWITCHER_GST_UTILS_H__
#define __SWITCHER_GST_UTILS_H__

#include <gst/gst.h>
#include <memory>
#include <string>
#include <vector>
#include "./bool-log.hpp"

namespace switcher {
namespace GstUtils {
GstElement* make_element(const gchar* class_name, GstElement** target_element);
bool link_static_to_request(GstElement* src, GstElement* sink);
bool link_static_to_request(GstPad* srcpad, GstElement* sink);
BoolLog check_pad_link_return(GstPadLinkReturn res);
void release_request_pad(GstPad* pad, gpointer user_data);
void unlink_pad(GstPad* pad);
void clean_element(GstElement* element);
void wait_state_changed(GstElement* bin);
BoolLog sync_state_with_parent(GstElement* element);
void set_element_property_in_bin(GstElement* bin,
                                 const gchar* factory_name,
                                 const gchar* property_name,
                                 gboolean property_value);
gchar* gvalue_serialize(const GValue* val);  // g_free after use
GSource* g_idle_add_full_with_context(GMainContext* context,
                                      gint priority,  // G_PRIORITY_DEFAULT_IDLE
                                      GSourceFunc function,
                                      gpointer data,
                                      GDestroyNotify notify);
GSource* g_timeout_add_to_context(guint interval,
                                  GSourceFunc function,
                                  gpointer data,
                                  GMainContext* context);
BoolLog apply_property_value(GObject* g_object_master,
                             GObject* g_object_slave,
                             const char* property_name);
void free_g_enum_values(GEnumValue* target_enum);
void element_factory_list_to_g_enum(GEnumValue* target_enum,
                                    GstElementFactoryListType type,
                                    GstRank minrank,
                                    bool insert_none_first = true,
                                    const std::vector<std::string>& black_list = {});
std::pair<std::vector<std::string> /*names*/, std::vector<std::string> /*nicks*/>
element_factory_list_to_pair_of_vectors(GstElementFactoryListType type,
                                        GstRank minrank,
                                        bool insert_none_first = true,
                                        const std::vector<std::string>& black_list = {});

void gst_element_deleter(GstElement* element);
gulong g_signal_connect_function(gpointer gobject,
                                 const gchar* signal,
                                 GCallback cb,
                                 gpointer user_data);
bool can_sink_caps(std::string factory_name, std::string caps);
GstElement* get_first_element_from_factory_name(GstBin* bin, const std::string& factory_name);
const GValue* get_gst_element_capability(const std::string& element_type,
                                         const std::string& capability,
                                         GstPadDirection direction);
std::vector<std::string> get_gst_element_capability_as_list(const std::string& element_type,
                                                            const std::string& capability,
                                                            GstPadDirection direction);
std::pair<int, int> get_gst_element_capability_as_range(const std::string& element_type,
                                                        const std::string& capability,
                                                        GstPadDirection direction);

}  // namespace GstUtils
}  // namespace switcher
#endif
