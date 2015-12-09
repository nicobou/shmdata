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
#include "./gst-shmdata-to-cb.hpp"
#include "./scope-exit.hpp"

namespace switcher {

GstShmdataToCb::GstShmdataToCb(const std::string &shmpath, on_caps_cb_t fun):
    pipe_(nullptr, nullptr),
    filter_cb_(fun) {
  GstElement *shmdatasrc = gst_element_factory_make("shmdatasrc", nullptr);
  GstElement *typefind = gst_element_factory_make("typefind", nullptr);
  On_scope_exit{ if (!is_valid_) {
      if (nullptr != shmdatasrc) gst_object_unref(shmdatasrc);
      if (nullptr != typefind) gst_object_unref(typefind);
    }};
  if (nullptr == shmdatasrc || nullptr == typefind){
    g_warning("GstShmdataToCb failled to create GStreamer element");
    return;
  } 
  g_signal_connect(typefind,
                   "have-type",
                   G_CALLBACK(on_caps),
                   this);
  g_object_set(G_OBJECT(shmdatasrc),
               "socket-path", shmpath.c_str(),
               nullptr);
  gst_bin_add_many(GST_BIN(pipe_.get_pipeline()),
                   shmdatasrc, typefind,
                   nullptr);
  if (!gst_element_link(shmdatasrc, typefind))
    return;
  pipe_.play(true);
  is_valid_ = true;
}


void GstShmdataToCb::on_handoff_cb(GstElement */*object*/,
                                  GstBuffer *buf,
                                  GstPad *pad,
                                  gpointer user_data) {
  //g_print("%s\n", __FUNCTION__);
  GstShmdataToCb *context = static_cast<GstShmdataToCb *>(user_data);
  //getting buffer infomation:
  GstMapInfo map;
  if (!gst_buffer_map (buf, &map, GST_MAP_READ)) {
    g_warning("gst_buffer_map failled: canceling audio buffer access");
    return;
  }
  On_scope_exit{gst_buffer_unmap (buf, &map);};
  std::unique_lock<std::mutex> lock(context->mtx_);
  for (auto &it: context->data_cbs_)
    it.second (map.data, map.size);
}

void GstShmdataToCb::on_caps(GstElement *typefind,
                             guint /*probability */ ,
                             GstCaps *caps,
                             gpointer user_data){
  GstShmdataToCb *context = static_cast<GstShmdataToCb *>(user_data);
  GstElement *fakesink = gst_element_factory_make("fakesink", nullptr);
  g_object_set(G_OBJECT(fakesink),
               "silent", TRUE,
               "signal-handoffs", TRUE,
               "sync", FALSE,
               nullptr);
  g_signal_connect(fakesink, "handoff", (GCallback)on_handoff_cb, context);
  gst_bin_add(GST_BIN(context->pipe_.get_pipeline()),
              fakesink);
  GstElement *filter = nullptr;
  if (context->filter_cb_){
    gchar *caps_str = gst_caps_to_string(caps);
    On_scope_exit{g_free(caps_str);};
    filter = context->filter_cb_(std::string(caps_str));
  }
  if (nullptr != filter) {
    gst_bin_add(GST_BIN(context->pipe_.get_pipeline()), filter);
    if (!gst_element_link_many(typefind, filter, fakesink, nullptr))
      g_warning("issue linking typefind with fakesink in GstShmdataToCb::on_caps");
    GstUtils::sync_state_with_parent(filter);
    GstUtils::sync_state_with_parent(fakesink);
  } else {
    if (!gst_element_link(typefind, fakesink))
      g_warning("issue linking typefind with fakesink in GstShmdataToCb::on_caps");
    GstUtils::sync_state_with_parent(fakesink);
  }
}

GstShmdataToCb::id_t GstShmdataToCb::add_cb(data_cb_t fun){
  if (!fun) return 0;
  std::unique_lock<std::mutex> lock(mtx_);
  auto res = ++counter_;
  data_cbs_[res] = fun;
  return res;
}

bool GstShmdataToCb::remove_cb(id_t cb_id){
  std::unique_lock<std::mutex> lock(mtx_);
  auto it = data_cbs_.find(cb_id);
  if (it == data_cbs_.end())
    return false;
  data_cbs_.erase(it);
  return true;
}

}  // namespace switcher
