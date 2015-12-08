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
  g_print("%s\n", __FUNCTION__);
  // GstShmdataToCb *context = static_cast<GstShmdataToCb *>(user_data);
  // auto current_time = jack_frame_time(context->jack_client_.get_raw());
  // GstCaps *caps = gst_pad_get_current_caps(pad);
  // if (nullptr == caps)
  //   return;
  // On_scope_exit {gst_caps_unref(caps);};
  // // gchar *string_caps = gst_caps_to_string(caps);
  // // On_scope_exit {if (nullptr != string_caps) g_free(string_caps);};
  // // g_print("on handoff, negotiated caps is %s\n", string_caps);
  // const GValue *val =
  //     gst_structure_get_value(gst_caps_get_structure(caps, 0),
  //                             "channels");
  // const int channels = g_value_get_int(val);
  // context->check_output_ports(channels);
  // //getting buffer infomation:
  // GstMapInfo map;
  // if (!gst_buffer_map (buf, &map, GST_MAP_READ)) {
  //   g_warning("gst_buffer_map failled: canceling audio buffer access");
  //   return;
  // }
  // On_scope_exit{gst_buffer_unmap (buf, &map);};
  // jack_nframes_t duration = map.size / (4 * channels);
  // // setting the smoothing value affecting (20 sec)
  // context->drift_observer_.set_smoothing_factor(
  //     (double)duration / (20.0 * (double)context->jack_client_.get_sample_rate()));
  // std::size_t new_size = 
  //     (std::size_t)context->drift_observer_.set_current_time_info(current_time, duration);
  // --context->debug_buffer_usage_;
  // if (0 == context->debug_buffer_usage_){
  //   g_debug("buffer load is %lu, ratio is %f\n",
  //           context->ring_buffers_[0].get_usage(),
  //           context->drift_observer_.get_ratio());
  //   context->debug_buffer_usage_ = 1000;
  // }
  // jack_sample_t *tmp_buf = (jack_sample_t *)map.data;
  // for (int i = 0; i < channels; ++i) {
  //   AudioResampler<jack_sample_t> resample(duration, new_size, tmp_buf, i, channels);
  //   auto emplaced =
  //       context->ring_buffers_[i].put_samples(
  //       new_size,
  //       [&resample]() {
  //         // return resample.zero_pole_get_next_sample();
  //         return resample.linear_get_next_sample();
  //       });
  //   if (emplaced != new_size)
  //     g_warning("overflow of %lu samples", new_size - emplaced);
  // }
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
  //handoff_handler_ =
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

}  // namespace switcher
