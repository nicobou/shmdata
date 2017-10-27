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

#include "./gst-shmdata-to-cb.hpp"
#include <gst/gst.h>
#include "./scope-exit.hpp"
#include "./string-utils.hpp"

namespace switcher {

GstShmdataToCb::GstShmdataToCb(const std::string& shmpath, on_caps_cb_t fun)
    : pipe_(nullptr, nullptr), filter_cb_(fun) {
  GstElement* shmdatasrc = gst_element_factory_make("shmdatasrc", nullptr);
  GstElement* typefind = gst_element_factory_make("typefind", nullptr);
  On_scope_exit {
    if (!is_valid_) {
      if (nullptr != shmdatasrc) gst_object_unref(shmdatasrc);
      if (nullptr != typefind) gst_object_unref(typefind);
    }
  };
  g_signal_connect(typefind, "have-type", G_CALLBACK(on_caps), this);
  g_object_set(G_OBJECT(shmdatasrc), "socket-path", shmpath.c_str(), nullptr);
  gst_bin_add_many(GST_BIN(pipe_.get_pipeline()), shmdatasrc, typefind, nullptr);
  if (!gst_element_link(shmdatasrc, typefind)) return;
  pipe_.play(true);
  is_valid_ = true;
}

void GstShmdataToCb::on_handoff_cb(GstElement* /*object*/,
                                   GstBuffer* buf,
                                   GstPad* /*pad*/,
                                   gpointer user_data) {
  GstShmdataToCb* context = static_cast<GstShmdataToCb*>(user_data);
  // getting buffer information:
  GstMapInfo map;
  if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
    return;
  }
  On_scope_exit { gst_buffer_unmap(buf, &map); };
  std::unique_lock<std::mutex> lock(context->mtx_);
  for (auto& it : context->data_cbs_) it.second(map.data, map.size);
}

void GstShmdataToCb::on_caps(GstElement* typefind,
                             guint /*probability */,
                             GstCaps* caps,
                             gpointer user_data) {
  GstShmdataToCb* context = static_cast<GstShmdataToCb*>(user_data);
  context->fakesink_ = gst_element_factory_make("fakesink", nullptr);
  g_object_set(G_OBJECT(context->fakesink_),
               "silent",
               TRUE,
               "signal-handoffs",
               TRUE,
               "sync",
               FALSE,
               nullptr);
  g_signal_connect(context->fakesink_, "handoff", (GCallback)on_handoff_cb, context);
  gst_bin_add(GST_BIN(context->pipe_.get_pipeline()), context->fakesink_);
  GstElement* filter = nullptr;
  if (context->filter_cb_) {
    gchar* caps_str = gst_caps_to_string(caps);
    On_scope_exit { g_free(caps_str); };
    filter = context->filter_cb_(std::string(caps_str));
  }
  if (nullptr != filter) {
    gst_bin_add(GST_BIN(context->pipe_.get_pipeline()), filter);
    gst_element_link_many(typefind, filter, context->fakesink_, nullptr);
    GstUtils::sync_state_with_parent(filter);
    GstUtils::sync_state_with_parent(context->fakesink_);
  } else {
    gst_element_link(typefind, context->fakesink_);
    GstUtils::sync_state_with_parent(context->fakesink_);
  }
}

GstShmdataToCb::id_t GstShmdataToCb::add_cb(data_cb_t fun) {
  if (!fun) return 0;
  std::unique_lock<std::mutex> lock(mtx_);
  auto res = ++counter_;
  data_cbs_[res] = fun;
  return res;
}

bool GstShmdataToCb::remove_cb(id_t cb_id) {
  std::unique_lock<std::mutex> lock(mtx_);
  auto it = data_cbs_.find(cb_id);
  if (it == data_cbs_.end()) return false;
  data_cbs_.erase(it);
  return true;
}

std::string GstShmdataToCb::get_caps() const {
  if (!fakesink_caps_.empty()) return fakesink_caps_;
  if (nullptr == fakesink_) {
    return std::string();
  }
  GstPad* pad = gst_element_get_static_pad(fakesink_, "sink");
  if (nullptr == pad) return fakesink_caps_;
  On_scope_exit { gst_object_unref(pad); };
  GstCaps* caps = gst_pad_get_current_caps(pad);
  if (nullptr == caps) return fakesink_caps_;
  On_scope_exit { gst_caps_unref(caps); };
  gchar* str = gst_caps_to_string(caps);
  if (nullptr == str) return fakesink_caps_;
  On_scope_exit { g_free(str); };
  fakesink_caps_ = std::string(str);
  return fakesink_caps_;
}

}  // namespace switcher
