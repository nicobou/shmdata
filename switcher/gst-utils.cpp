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

#include "./gst-utils.hpp"
#include <algorithm>
#include <string>
#include "./scope-exit.hpp"
#include "bool-log.hpp"

namespace switcher {
GstElement* GstUtils::make_element(const gchar* class_name, GstElement** target_element) {
  GstElement* res = gst_element_factory_make(class_name, nullptr);
  if (res == nullptr) {
    return nullptr;
  }
  if (target_element) *target_element = res;
  return res;
}

bool GstUtils::link_static_to_request(GstElement* src, GstElement* sink) {
  GstPad* srcpad = gst_element_get_static_pad(src, "src");
  GstPad* sinkpad =
      gst_element_get_compatible_pad(sink,
                                     srcpad,
                                     nullptr);  // const GstCaps *caps to use as a filter
  On_scope_exit {
    if (GST_IS_PAD(src)) gst_object_unref(srcpad);
    if (GST_IS_PAD(sinkpad)) gst_object_unref(sinkpad);
  };
  return GstUtils::check_pad_link_return(gst_pad_link(srcpad, sinkpad));
}

bool GstUtils::link_static_to_request(GstPad* srcpad, GstElement* sink) {
  GstPad* sinkpad =
      gst_element_get_compatible_pad(sink,
                                     srcpad,
                                     nullptr);  // const GstCaps *caps to use as a filter
  On_scope_exit {
    if (GST_IS_PAD(sinkpad)) gst_object_unref(sinkpad);
  };

  return GstUtils::check_pad_link_return(gst_pad_link(srcpad, sinkpad));
}

BoolLog GstUtils::check_pad_link_return(GstPadLinkReturn res) {
  if (res == GST_PAD_LINK_OK) return BoolLog(true);
  std::string log;
  switch (res) {
    case GST_PAD_LINK_WRONG_HIERARCHY:
      log = "GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_HIERARCHY";
      break;
    case GST_PAD_LINK_WAS_LINKED:
      log = "GstUtils::check_pad_link_return - GST_PAD_LINK_WAS_LINKED";
      break;
    case GST_PAD_LINK_WRONG_DIRECTION:
      log = "GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_DIRECTION";
      break;
    case GST_PAD_LINK_NOFORMAT:
      log = "GstUtils::check_pad_link_return - GST_PAD_LINK_NOFORMAT";
      break;
    case GST_PAD_LINK_NOSCHED:
      log = "GstUtils::check_pad_link_return - GST_PAD_LINK_NOSCHED";
      break;
    case GST_PAD_LINK_REFUSED:
      log = "GstUtils::check_pad_link_return - GST_PAD_LINK_REFUSED";
      break;
    default:
      log = "GstUtils::check_pad_link_return - UNKNOWN ERROR";
  }
  return BoolLog(false, log);
}

void GstUtils::unlink_pad(GstPad* pad) {
  GstPad* peer;
  if ((peer = gst_pad_get_peer(pad))) {
    if (gst_pad_get_direction(pad) == GST_PAD_SRC)
      gst_pad_unlink(pad, peer);
    else
      gst_pad_unlink(peer, pad);
    gst_object_unref(peer);
  }
  gst_object_unref(pad);  // for iterator
}

void GstUtils::release_request_pad(GstPad* pad, gpointer user_data) {
  // checking if the pad has been requested and releasing it needed
  GstPadTemplate* pad_templ = gst_pad_get_pad_template(pad);
  if (nullptr != pad_templ && GST_PAD_TEMPLATE_PRESENCE(pad_templ) == GST_PAD_REQUEST) {
    gst_element_release_request_pad((GstElement*)user_data, pad);
    gst_object_unref(pad);  // release does not free
  }
  // gst_object_unref(pad_templ);  // bug in gst 0.10 ?
  gst_object_unref(pad);  // for iterator
}

void GstUtils::clean_element(GstElement* element) {
  if (nullptr == element) return;
  if (!GST_IS_ELEMENT(element)) {
    return;
  }
  if (GST_STATE_CHANGE_FAILURE == GST_STATE_RETURN(element)) {
    return;
  }
  gst_element_set_state(element, GST_STATE_NULL);
  GstObject* parent = gst_element_get_parent(element);
  On_scope_exit {
    if (parent) gst_object_unref(parent);
  };
  if (GST_IS_BIN(parent)) {
    gst_bin_remove(GST_BIN_CAST(parent), element);
  } else {
    if (((GObject*)element)->ref_count > 0) gst_object_unref(element);
  }
}

void GstUtils::wait_state_changed(GstElement* bin) {
  if (!GST_IS_BIN(bin)) {
    return;
  }
  GValue val = G_VALUE_INIT;
  g_value_init(&val, G_TYPE_BOOLEAN);

  g_object_get_property(G_OBJECT(bin), "async-handling", &val);

  if (g_value_get_boolean(&val) == FALSE)
    while (GST_STATE(bin) != GST_STATE_TARGET(bin)) {
      // warning this may be blocking
      gst_element_get_state(bin, nullptr, nullptr, GST_CLOCK_TIME_NONE);
    }
  g_value_unset(&val);
  return;
}

BoolLog GstUtils::sync_state_with_parent(GstElement* element) {
  if (!GST_IS_ELEMENT(element)) {
    return BoolLog(false, "GstUtils::sync_state_with_parent, arg is not an element");
  }

  GstElement* parent = GST_ELEMENT(GST_ELEMENT_PARENT(element));
  if (GST_IS_ELEMENT(parent)) {
    if (GST_STATE(parent) != GST_STATE_TARGET(parent))
      gst_element_set_state(element, GST_STATE_TARGET(parent));
    else
      gst_element_sync_state_with_parent(element);
  } else
    return BoolLog(false, "GstUtils::sync_state_with_parent, cannot sync an orphan element");
  return BoolLog(true);
}

void GstUtils::set_element_property_in_bin(GstElement* bin,
                                           const gchar* factory_name,
                                           const gchar* property_name,
                                           gboolean property_value) {
  if (!GST_IS_BIN(bin)) return;

  if (g_list_length(GST_BIN_CHILDREN(GST_BIN(bin))) > 0) {
    GList *child = nullptr, *children = GST_BIN_CHILDREN(GST_BIN(bin));
    for (child = children; child != nullptr; child = g_list_next(child)) {
      GstElement* current_element = GST_ELEMENT(child->data);
      GstElementFactory* factory = gst_element_get_factory(current_element);
      if (g_strcmp0(factory_name, gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(factory))) == 0) {
        g_object_set(G_OBJECT(current_element), property_name, property_value, nullptr);
      }

      if (GST_IS_BIN(current_element)) {  // recursive
        GstUtils::set_element_property_in_bin(
            current_element, factory_name, property_name, property_value);
      }
    }
  }
}

GstElement* GstUtils::get_first_element_from_factory_name(GstBin* bin,
                                                          const std::string& factory_name) {
  if (!GST_IS_BIN(bin)) {
    return nullptr;
  }

  if (g_list_length(GST_BIN_CHILDREN(bin)) == 0) {
    return nullptr;
  }

  GList *child = nullptr, *children = GST_BIN_CHILDREN(GST_BIN(bin));
  for (child = children; child != nullptr; child = g_list_next(child)) {
    GstElement* current_element = GST_ELEMENT(child->data);
    GstElementFactory* factory = gst_element_get_factory(current_element);
    if (factory_name == gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(factory))) {
      return current_element;
    }
  }
  return nullptr;
}

gchar* GstUtils::gvalue_serialize(const GValue* val) {
  if (!G_IS_VALUE(val)) return nullptr;
  gchar* val_str;
  if (G_VALUE_TYPE(val) == G_TYPE_STRING)
    val_str = g_strdup(g_value_get_string(val));
  else
    val_str = gst_value_serialize(val);
  return val_str;
}

GSource* GstUtils::g_idle_add_full_with_context(GMainContext* context,
                                                gint priority,
                                                GSourceFunc function,
                                                gpointer data,
                                                GDestroyNotify notify) {
  GSource* source;
  if (function == nullptr) return nullptr;
  source = g_idle_source_new();
  if (priority != G_PRIORITY_DEFAULT_IDLE) g_source_set_priority(source, priority);
  g_source_set_callback(source, function, data, notify);
  g_source_attach(source, context);
  g_source_unref(source);
  return source;
}

GSource* GstUtils::g_timeout_add_to_context(guint interval,
                                            GSourceFunc function,
                                            gpointer data,
                                            GMainContext* context) {
  GSource* source;
  g_return_val_if_fail(function != nullptr, 0);
  source = g_timeout_source_new(interval);
  g_source_set_callback(source, function, data, nullptr);
  /*guint id = */ g_source_attach(source, context);
  g_source_unref(source);
  return source;
}

BoolLog GstUtils::apply_property_value(GObject* g_object_master,
                                       GObject* g_object_slave,
                                       const char* property_name) {
  if (g_object_master == nullptr || g_object_slave == nullptr)
    return BoolLog(false, "g_object_master or g_object_slave is nullptr");

  if (!G_IS_OBJECT(g_object_master) || !G_IS_OBJECT(g_object_slave))
    return BoolLog(false, "_object_master or g_object_slave is not a glib object");

  GParamSpec* pspec_master = g_object_class_find_property(
      G_OBJECT_CLASS(G_OBJECT_GET_CLASS(g_object_master)), property_name);
  if (pspec_master == nullptr) {
    return BoolLog(false, std::string(property_name) + ": property not found for master");
  }

  GParamSpec* pspec_slave = g_object_class_find_property(
      G_OBJECT_CLASS(G_OBJECT_GET_CLASS(g_object_slave)), property_name);
  if (pspec_slave == nullptr) {
    return BoolLog(false, std::string(property_name) + ": property not found for slave");
  }

  if (pspec_master->value_type != pspec_slave->value_type) {
    return BoolLog(false, "master and slave properties has different type, canont apply");
  }

  GValue val = G_VALUE_INIT;
  g_value_init(&val, pspec_master->value_type);

  g_object_get_property(g_object_master, property_name, &val);

  g_object_set_property(g_object_slave, property_name, &val);
  g_value_unset(&val);
  return BoolLog(true);
}

void GstUtils::free_g_enum_values(GEnumValue* target_enum) {
  if (nullptr == target_enum) return;
  gint i = 0;
  while (nullptr != target_enum[i].value_name) {
    g_free((gpointer)target_enum[i].value_name);
    g_free((gpointer)target_enum[i].value_nick);
    i++;
  }
}

void GstUtils::element_factory_list_to_g_enum(GEnumValue* target_enum,
                                              GstElementFactoryListType type,
                                              GstRank minrank,
                                              bool insert_none_first,
                                              const std::vector<std::string>& black_list) {
  GList* element_list = gst_element_factory_list_get_elements(type, minrank);

  GList* iter = element_list;
  gint i = 0;
  if (insert_none_first) {
    target_enum[0].value = 0;
    target_enum[0].value_name = g_strdup("None");
    target_enum[0].value_nick = g_strdup("None");
    i++;
  }
  while (iter != nullptr) {
    if (black_list.end() == std::find(black_list.begin(),
                                      black_list.end(),
                                      gst_plugin_feature_get_name((GstPluginFeature*)iter->data))) {
      target_enum[i].value = i;
      target_enum[i].value_name =
          g_strdup(gst_element_factory_get_longname((GstElementFactory*)iter->data));
      target_enum[i].value_nick =
          g_strdup(gst_plugin_feature_get_name((GstPluginFeature*)iter->data));
      i++;
    }
    iter = g_list_next(iter);
  }
  target_enum[i].value = 0;
  target_enum[i].value_name = nullptr;
  target_enum[i].value_nick = nullptr;
  gst_plugin_feature_list_free(element_list);
}

std::pair<std::vector<std::string> /*names*/, std::vector<std::string> /*nicks*/>
GstUtils::element_factory_list_to_pair_of_vectors(GstElementFactoryListType type,
                                                  GstRank minrank,
                                                  bool insert_none_first,
                                                  const std::vector<std::string>& black_list) {
  std::vector<std::string> names{};
  std::vector<std::string> nicks{};
  GList* element_list = gst_element_factory_list_get_elements(type, minrank);

  GList* iter = element_list;
  gint i = 0;
  if (insert_none_first) {
    names.emplace_back("None");
    nicks.emplace_back("None");
    i++;
  }
  while (iter != nullptr) {
    if (black_list.end() == std::find(black_list.begin(),
                                      black_list.end(),
                                      gst_plugin_feature_get_name((GstPluginFeature*)iter->data))) {
      names.emplace_back(gst_element_factory_get_longname((GstElementFactory*)iter->data));
      nicks.emplace_back(gst_plugin_feature_get_name((GstPluginFeature*)iter->data));
      i++;
    }
    iter = g_list_next(iter);
  }
  gst_plugin_feature_list_free(element_list);
  return std::make_pair(names, nicks);
}

void GstUtils::gst_element_deleter(GstElement* element) {
  if (nullptr == element) {
    return;
  }
  if (!G_IS_OBJECT(element)) {
    return;
  }
  // unref if ownership has not been taken by a parent
  if (nullptr == GST_OBJECT_PARENT(element)) {
    if (((GObject*)element)->ref_count > 0) gst_object_unref(element);
  }
}

// g_signal_connect is actually a macro, so wrapping it for use with std::bind
gulong GstUtils::g_signal_connect_function(gpointer gobject,
                                           const gchar* signal,
                                           GCallback cb,
                                           gpointer user_data) {
  return g_signal_connect(gobject, signal, cb, user_data);
}

bool GstUtils::can_sink_caps(std::string factory_name, std::string caps) {
  if (caps.empty()) {
    return false;
  }

  GstCaps* caps_ptr = gst_caps_from_string(caps.c_str());
  On_scope_exit { gst_caps_unref(caps_ptr); };

  GstElementFactory* factory = gst_element_factory_find(factory_name.c_str());
  if (nullptr == factory) {
    return false;
  }
  On_scope_exit { gst_object_unref(factory); };

  if (!gst_element_factory_can_sink_all_caps(factory, caps_ptr)) return false;
  return true;
}

const GValue* GstUtils::get_gst_element_capability(const std::string& element_type,
                                                   const std::string& capability,
                                                   GstPadDirection direction) {
  GstElementFactory* factory = gst_element_factory_find(element_type.c_str());
  On_scope_exit { gst_object_unref(factory); };
  const GList* list = gst_element_factory_get_static_pad_templates(factory);
  while (nullptr != list) {
    GstStaticPadTemplate* templ = static_cast<GstStaticPadTemplate*>(list->data);
    if (templ->direction != GST_PAD_UNKNOWN && templ->direction == direction) {
      GstCaps* caps = gst_static_pad_template_get_caps(templ);
      On_scope_exit { gst_caps_unref(caps); };
      GstStructure* structure = gst_caps_get_structure(caps, 0);
      return gst_structure_get_value(structure, capability.c_str());
    }
    list = g_list_next(list);
  }
  return nullptr;
}

std::vector<std::string> GstUtils::get_gst_element_capability_as_list(
    const std::string& element_type, const std::string& capability, GstPadDirection direction) {
  std::vector<std::string> values;
  const GValue* value_list = get_gst_element_capability(element_type, capability, direction);
  if (value_list && GST_VALUE_HOLDS_LIST(value_list)) {
    for (guint j = 0; j < gst_value_list_get_size(value_list); ++j) {
      values.emplace_back(g_value_get_string(gst_value_list_get_value(value_list, j)));
    }
  }
  return values;
}

std::pair<int, int> GstUtils::get_gst_element_capability_as_range(const std::string& element_type,
                                                                  const std::string& capability,
                                                                  GstPadDirection direction) {
  std::pair<int, int> range;
  const GValue* value = get_gst_element_capability(element_type, capability, direction);
  if (value && GST_VALUE_HOLDS_INT_RANGE(value)) {
    range.first = gst_value_get_int_range_min(value);
    range.second = gst_value_get_int_range_max(value);
  }
  return range;
}

}  // namespace switcher
