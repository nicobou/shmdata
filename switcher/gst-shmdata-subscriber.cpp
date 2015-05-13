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
                                           on_byte_monitor_t on_byte_monitor_cb,
                                           int polling_ms) :
    element_(element),
    on_caps_cb_(on_caps_cb),
    on_byte_monitor_cb_(on_byte_monitor_cb),
    polling_ms_(polling_ms),
    byte_monitor_ (GST_IS_ELEMENT(element) ?
                   std::async(std::launch::async, [this](){byte_monitor();})
                   : std::future<void>()) {
}

GstShmdataSubscriber::~GstShmdataSubscriber(){
  quit_.store(true);
  if (byte_monitor_.valid()) 
    byte_monitor_.get();
}

void GstShmdataSubscriber::on_caps_cb(GObject *gobject, GParamSpec *pspec, gpointer user_data){
  GstShmdataSubscriber *context = static_cast<GstShmdataSubscriber *>(user_data);
  GValue val = G_VALUE_INIT;
  g_value_init(&val, pspec->value_type);
  g_object_get_property(gobject, "caps", &val);
  context->caps_ = std::string(g_value_get_string(&val));
  g_print("caps received: %s"
          // ", category: %s\n"
          ,context->caps_.c_str()
          //,ShmdataCategory::get_category(context->caps_).c_str()
          );
  g_value_unset(&val);
}

void GstShmdataSubscriber::byte_monitor(){
  while(!quit_.load()){
    std::this_thread::sleep_for(std::chrono::milliseconds (300));
    GValue val = G_VALUE_INIT;
    g_value_init(&val, G_TYPE_UINT64);
    g_object_get_property(G_OBJECT(element_), "bytes", &val);
    g_print("bytes %lu \n", g_value_get_uint64(&val));
    g_value_unset(&val);
  }
}

}  // namespace switcher
