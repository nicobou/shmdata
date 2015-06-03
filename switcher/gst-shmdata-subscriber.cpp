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

#include <gst/gst.h>
#include "./gst-shmdata-subscriber.hpp"

namespace switcher {

GstShmdataSubscriber::GstShmdataSubscriber(GstElement *element,
                                           on_caps_cb_t on_caps_cb,
                                           on_byte_monitor_t on_byte_monitor_cb) :
    element_(element),
    on_caps_cb_(on_caps_cb),
    on_byte_monitor_cb_(on_byte_monitor_cb),
    ptask_ ([this](){this->byte_monitor();},
            std::chrono::milliseconds (1000)) {
  if (!GST_IS_ELEMENT(element_)){
    g_warning("cannot monitor gstshmdata metadata, not a GstElement");
    return;
  }
  g_signal_connect(G_OBJECT(element_),
                   "notify::caps",
                   G_CALLBACK(GstShmdataSubscriber::on_caps_cb),
                   this);
  notify_caps();
}

void GstShmdataSubscriber::on_caps_cb(GObject *gobject,
                                      GParamSpec */*pspec*/,
                                      gpointer user_data){
  GstShmdataSubscriber *context = static_cast<GstShmdataSubscriber *>(user_data);
  if (!context->on_caps_cb_) {
    g_debug("got caps, but no caps callback in GstShmdataSubscriber");
    return;
  }
  context->notify_caps();
}

void GstShmdataSubscriber::notify_caps(){
  GValue val = G_VALUE_INIT;
  g_value_init(&val, G_TYPE_STRING);
  g_object_get_property(G_OBJECT(element_), "caps", &val);
  if (nullptr != g_value_get_string(&val))
    on_caps_cb_(g_value_get_string(&val));
  g_value_unset(&val);
}

void GstShmdataSubscriber::byte_monitor(){
  if (!on_byte_monitor_cb_) {
    g_debug("no byte monitor callback, not monitoring");
    return;
  }
  GValue val = G_VALUE_INIT;
  g_value_init(&val, G_TYPE_UINT64);
  g_object_get_property(G_OBJECT(element_), "bytes", &val);
  on_byte_monitor_cb_(g_value_get_uint64(&val));
  g_value_unset(&val);
}

}  // namespace switcher
