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

//#include <unistd.h>  // sleep
#include <string>
#include <algorithm>
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"

namespace switcher {
bool
GstUtils::make_element(const gchar *class_name,
                       GstElement **target_element) {
  *target_element = gst_element_factory_make(class_name, nullptr);
  if (*target_element == nullptr) {
    g_debug("gstreamer element class %s cannot be instanciated",
            class_name);
    return false;
  }
  return true;
}



bool GstUtils::link_static_to_request(GstElement * src, GstElement *sink) {
  GstPad *srcpad = gst_element_get_static_pad(src, "src");
  GstPad *sinkpad = gst_element_get_compatible_pad(sink,
                                                   srcpad,
                                                   nullptr);  // const GstCaps *caps to use as a filter
  bool res = GstUtils::check_pad_link_return(gst_pad_link(srcpad, sinkpad));
  if (GST_IS_PAD(src))
    gst_object_unref(srcpad);

  if (GST_IS_PAD(sinkpad))
    gst_object_unref(sinkpad);

  return res;
}

bool GstUtils::link_static_to_request(GstPad * srcpad, GstElement *sink) {
  GstPad *sinkpad = gst_element_get_compatible_pad(sink,
                                                   srcpad,
                                                   nullptr);  // const GstCaps *caps to use as a filter
  bool res = GstUtils::check_pad_link_return(gst_pad_link(srcpad, sinkpad));

  if (GST_IS_PAD(sinkpad))
    gst_object_unref(sinkpad);

  return res;
}

bool GstUtils::check_pad_link_return(GstPadLinkReturn res) {
  if (res == GST_PAD_LINK_OK)
    return true;

  switch (res) {
    case GST_PAD_LINK_WRONG_HIERARCHY:
      g_debug
          ("GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_HIERARCHY");
      break;
    case GST_PAD_LINK_WAS_LINKED:
      g_debug("GstUtils::check_pad_link_return - GST_PAD_LINK_WAS_LINKED");
      break;
    case GST_PAD_LINK_WRONG_DIRECTION:
      g_debug
          ("GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_DIRECTION");
      break;
    case GST_PAD_LINK_NOFORMAT:
      g_debug("GstUtils::check_pad_link_return - GST_PAD_LINK_NOFORMAT");
      break;
    case GST_PAD_LINK_NOSCHED:
      g_debug("GstUtils::check_pad_link_return - GST_PAD_LINK_NOSCHED");
      break;
    case GST_PAD_LINK_REFUSED:
      g_debug("GstUtils::check_pad_link_return - GST_PAD_LINK_REFUSED");
      break;
    default:
      g_debug("GstUtils::check_pad_link_return - UNKNOWN ERROR");
  }
  return false;
}

void GstUtils::unlink_pad(GstPad *pad) {
  GstPad *peer;
  if ((peer = gst_pad_get_peer(pad))) {
    if (gst_pad_get_direction(pad) == GST_PAD_SRC)
      gst_pad_unlink(pad, peer);
    else
      gst_pad_unlink(peer, pad);
    gst_object_unref(peer);
  }
  gst_object_unref(pad);  // for iterator
}

void GstUtils::release_request_pad(GstPad *pad, gpointer user_data) {
  // checking if the pad has been requested and releasing it needed
  GstPadTemplate *pad_templ = gst_pad_get_pad_template(pad);
  if (nullptr != pad_templ
      && GST_PAD_TEMPLATE_PRESENCE(pad_templ) == GST_PAD_REQUEST) {
    gst_element_release_request_pad((GstElement *)user_data,
                                    pad);
    gst_object_unref(pad);  // release does not free
  }
  //gst_object_unref(pad_templ);  // bug in gst 0.10 ?
  gst_object_unref(pad);  // for iterator
}

void GstUtils::clean_element(GstElement *element) {
  if (nullptr == element)
    return;
  if (!GST_IS_ELEMENT(element)) {
    g_warning("%s failed (not a gst element)", __FUNCTION__);
    return;
  }
  if (GST_STATE_CHANGE_FAILURE == GST_STATE_RETURN(element)) {
    g_warning("%s failed (state error)", __FUNCTION__);
    return;
  }
  // if (GST_IS_BIN(element))
  //   g_debug("%d, %d, %d, state return %d", GST_STATE(element),
  //           GST_STATE_TARGET(element), GST_STATE_PENDING(element),
  //           GST_STATE_RETURN(element));
  
  // FIXME {  // unlinking pads
  //   GstIterator *pad_iter;
  //   pad_iter = gst_element_iterate_pads(element);
  //   gst_iterator_foreach(pad_iter, (GFunc) GstUtils::unlink_pad, nullptr);
  //   gst_iterator_free(pad_iter);
  // }
  
  // FIXME {  // releasing request pads
  //   GstIterator *pad_iter;
  //   pad_iter = gst_element_iterate_pads(element);
  //   gst_iterator_foreach(pad_iter, (GFunc) GstUtils::release_request_pad, element);
  //   gst_iterator_free(pad_iter);
  // }

  // GstState state = GST_STATE_TARGET(element);
  // if (state != GST_STATE_NULL) {
  //   if (GST_STATE_CHANGE_ASYNC ==
  //       gst_element_set_state(element, GST_STATE_NULL)) {
  //     while (GST_STATE(element) != GST_STATE_NULL) {
  //       // warning this may be blocking
  //       gst_element_get_state(element, nullptr, nullptr,
  //                             GST_CLOCK_TIME_NONE);
  //     }
  //   }
  // }
  // if (GST_IS_BIN(gst_element_get_parent(element)))
  //   gst_bin_remove(GST_BIN(gst_element_get_parent(element)), element);
  // else
    if (!GST_IS_BIN(gst_element_get_parent(element))
        && ((GObject *) element)->ref_count > 0)
    gst_object_unref(element);
}

void GstUtils::wait_state_changed(GstElement *bin) {
  if (!GST_IS_BIN(bin)) {
    g_warning("GstUtils::wait_state_changed not a bin");
    return;
  }
  GValue val = G_VALUE_INIT;
  g_value_init(&val, G_TYPE_BOOLEAN);

  g_object_get_property(G_OBJECT(bin), "async-handling", &val);

  if (g_value_get_boolean(&val) == FALSE)
    while (GST_STATE(bin) != GST_STATE_TARGET(bin)) {
      g_debug("GstUtils::wait_state_changed, from %s to %s",
              gst_element_state_get_name(GST_STATE(bin)),
              gst_element_state_get_name(GST_STATE_TARGET(bin)));
      // warning this may be blocking
      gst_element_get_state(bin, nullptr, nullptr, GST_CLOCK_TIME_NONE);
    }
  g_value_unset(&val);
  return;
}

void GstUtils::sync_state_with_parent(GstElement *element) {
  if (!GST_IS_ELEMENT(element)) {
    g_debug("GstUtils::sync_state_with_parent, arg is not an element");
    return;
  }

  GstElement *parent = GST_ELEMENT(GST_ELEMENT_PARENT(element));
  if (GST_IS_ELEMENT(parent)) {
    if (GST_STATE(parent) != GST_STATE_TARGET(parent))
      gst_element_set_state(element, GST_STATE_TARGET(parent));
    else
      gst_element_sync_state_with_parent(element);
  }
  else
    g_warning
        ("GstUtils::sync_state_with_parent, cannot sync an orphan element");
}

void
GstUtils::set_element_property_in_bin(GstElement *bin,
                                      const gchar *factory_name,
                                      const gchar *property_name,
                                      gboolean property_value) {
  if (!GST_IS_BIN(bin))
    return;

  if (g_list_length(GST_BIN_CHILDREN(GST_BIN(bin))) > 0) {
    GList *child = nullptr, *children = GST_BIN_CHILDREN(GST_BIN(bin));
    for (child = children; child != nullptr; child = g_list_next(child)) {
      GstElement *current_element = GST_ELEMENT(child->data);
      GstElementFactory *factory = gst_element_get_factory(current_element);
      // g_print ("The '%s' element is a member of the category %s.\n"
      //  "Description: %s\n",
      //  gst_plugin_feature_get_name (GST_PLUGIN_FEATURE (sub_factory)),
      //  gst_element_factory_get_klass (sub_factory),
      //  gst_element_factory_get_description (sub_factory));
      if (g_strcmp0(factory_name,
                    gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(factory)))
          == 0) {
        g_debug("GstUtils: setting property for %s", factory_name);
        g_object_set(G_OBJECT(current_element), property_name,
                     property_value, nullptr);
      }

      if (GST_IS_BIN(current_element)) {  // recursive
        GstUtils::set_element_property_in_bin(current_element,
                                              factory_name,
                                              property_name,
                                              property_value);
      }
      
    }
  }
}

GstElement *
GstUtils::get_first_element_from_factory_name(GstBin *bin,
                                              const std::string &factory_name) {
  if (!GST_IS_BIN(bin)) {
    g_warning("%s: first argument is not a bin", __FUNCTION__);
    return nullptr;
  }

  if (g_list_length(GST_BIN_CHILDREN(bin)) == 0) {
    g_warning("%s: bin has no child", __FUNCTION__);
    return nullptr;
  }

  GList *child = nullptr, *children = GST_BIN_CHILDREN(GST_BIN(bin));
  for (child = children; child != nullptr; child = g_list_next(child)) {
    GstElement *current_element = GST_ELEMENT(child->data);
    GstElementFactory *factory = gst_element_get_factory(current_element);
    if (factory_name == gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(factory))) {
      return current_element;
    }
  }
  g_warning("%s: no element found for %s", __FUNCTION__, factory_name.c_str());
  return nullptr;
}

gchar *GstUtils::gvalue_serialize(const GValue *val) {
  if (!G_IS_VALUE(val))
    return nullptr;
  gchar *val_str;
  if (G_VALUE_TYPE(val) == G_TYPE_STRING)
    val_str = g_strdup(g_value_get_string(val));
  else
    val_str = gst_value_serialize(val);
  return val_str;
}

GSource *
GstUtils::g_idle_add_full_with_context(GMainContext *context,
                                       gint priority,
                                       GSourceFunc function,
                                       gpointer data,
                                       GDestroyNotify notify) {
  GSource *source;
  if (function == nullptr)
    return nullptr;
  source = g_idle_source_new();
  if (priority != G_PRIORITY_DEFAULT_IDLE)
    g_source_set_priority(source, priority);
  g_source_set_callback(source, function, data, notify);
  g_source_attach(source, context);
  g_source_unref(source);
  return source;
}

GSource *GstUtils::g_timeout_add_to_context(guint interval,
                                            GSourceFunc function,
                                            gpointer data,
                                            GMainContext *context) {
  GSource *source;
  g_return_val_if_fail(function != nullptr, 0);
  source = g_timeout_source_new(interval);
  g_source_set_callback(source, function, data, nullptr);
  /*guint id = */ g_source_attach(source, context);
  g_source_unref(source);
  return source;
}

bool
GstUtils::apply_property_value(GObject *g_object_master,
                               GObject *g_object_slave,
                               const char *property_name) {
  if (g_object_master == nullptr || g_object_slave == nullptr)
    return false;

  if (!G_IS_OBJECT(g_object_master) || !G_IS_OBJECT(g_object_slave))
    return false;

  GParamSpec *pspec_master =
      g_object_class_find_property(G_OBJECT_CLASS
                                   (G_OBJECT_GET_CLASS(g_object_master)),
                                   property_name);
  if (pspec_master == nullptr) {
    g_debug("property %s not found for master ", property_name);
    return false;
  }

  GParamSpec *pspec_slave =
      g_object_class_find_property(G_OBJECT_CLASS
                                   (G_OBJECT_GET_CLASS(g_object_slave)),
                                   property_name);
  if (pspec_slave == nullptr) {
    g_debug("property %s, not found for slave", property_name);
    return false;
  }

  if (pspec_master->value_type != pspec_slave->value_type) {
    g_debug("master and slave properties has different type, canont apply");
    return false;
  }

  GValue val = G_VALUE_INIT;
  g_value_init(&val, pspec_master->value_type);

  g_object_get_property(g_object_master, property_name, &val);

  g_object_set_property(g_object_slave, property_name, &val);
  g_value_unset(&val);
  return true;
}

void
GstUtils::free_g_enum_values(GEnumValue *target_enum) {
  if (nullptr == target_enum) {
    g_warning("cannot free a nullptr enum");
    return;
  }
  gint i = 0;
  while (nullptr != target_enum[i].value_name) {
    g_free((gpointer)target_enum[i].value_name);
    g_free((gpointer)target_enum[i].value_nick);
    i++;    
  }
}

void
GstUtils::element_factory_list_to_g_enum(GEnumValue *target_enum,
                                         GstElementFactoryListType type,
                                         GstRank minrank,
                                         bool insert_none_first,
                                         const std::vector<std::string> &black_list) {
  GList *element_list =
      gst_element_factory_list_get_elements(type, minrank);

  GList *iter = element_list;
  gint i = 0;
  if (insert_none_first) {
    target_enum[0].value = 0;
    target_enum[0].value_name = g_strdup("None");
    target_enum[0].value_nick = g_strdup("None");
    i++;
  }
  while (iter != nullptr) {
    if (black_list.end() == std::find(
            black_list.begin(),
            black_list.end(), 
            gst_plugin_feature_get_name((GstPluginFeature *) iter->data))){
      target_enum[i].value = i;
      target_enum[i].value_name =
          g_strdup(gst_element_factory_get_longname((GstElementFactory *) iter->data));
      target_enum[i].value_nick =
          g_strdup(gst_plugin_feature_get_name((GstPluginFeature *) iter->data));
      i++;
    }
    iter = g_list_next(iter);
  }
  target_enum[i].value = 0;
  target_enum[i].value_name = nullptr;
  target_enum[i].value_nick = nullptr;
  gst_plugin_feature_list_free(element_list);
}

void GstUtils::gst_element_deleter(GstElement *element) {
  if (nullptr == element) {
    g_warning("%s is trying to delete a null element", __FUNCTION__);
    return;
  }
  if (!G_IS_OBJECT(element)) {
    g_warning("%s is trying to delete something not a GObject",
	      __FUNCTION__);
    return;
  }
    
  // delete if ownership has not been taken by a parent
  if (nullptr == GST_OBJECT_PARENT(element)) {
    if (((GObject *) element)->ref_count > 0)
      gst_object_unref(element);
  } else {
    GstUtils::clean_element(element);
  }
}

// g_signal_connect is actually a macro, so wrapping it for use with std::bind
gulong
GstUtils::g_signal_connect_function(gpointer gobject,
                                    const gchar *signal,
                                    GCallback cb, gpointer user_data) {
  return g_signal_connect(gobject, signal, cb, user_data);
}

bool GstUtils::can_sink_caps(std::string factory_name, std::string caps) {
  if (caps.empty()) {
    g_warning("%s: input caps string is empty, returning false",
              __FUNCTION__);
    return false;
  }

  GstCaps *caps_ptr = gst_caps_from_string(caps.c_str());
  On_scope_exit {
    gst_caps_unref(caps_ptr);
  };

  GstElementFactory *factory =
      gst_element_factory_find(factory_name.c_str());
  if (nullptr == factory) {
    g_warning("%s: factory %s cannot be found, returning false",
              __FUNCTION__, factory_name.c_str());
    return false;
  }
  On_scope_exit {
    gst_object_unref(factory);
  };

  if (!gst_element_factory_can_sink_all_caps(factory, caps_ptr))
    return false;
  return true;
}

}  // namespace switcher
