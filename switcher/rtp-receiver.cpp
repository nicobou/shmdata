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

#include "./rtp-receiver.hpp"
#include "./rtp-session2.hpp"
#include "./scope-exit.hpp"

namespace switcher {

RTPReceiver::RTPReceiver(RtpSession2* session,
                         const std::string& rtpshmpath,
                         configure_shmsink_cb_t cb,
                         bool decompress)
    : session_(session),
      rtpshmpath_(rtpshmpath),
      shmdatasrc_(gst_element_factory_make("shmdatasrc", nullptr)),
      typefind_(gst_element_factory_make("typefind", nullptr)),
      decompress_(decompress),
      configure_shmsink_cb_(cb),
      decodebin_(
          session_->gst_pipeline_.get(),
          [this](GstElement* el, const std::string& media_type, const std::string& media_label) {
            if (configure_shmsink_cb_) {
              configure_shmsink_cb_(el, media_type, media_label);
            } else {
              std::string path = std::string("/tmp/") + media_type + "-" + media_label;
              g_object_set(G_OBJECT(el), "socket-path", path.c_str(), nullptr);
            }
          },
          [this]() {  // FIXME warning("discarding uncomplete custom frame due to a network loss");
          },
          decompress_) {
  // monitoring rtp-session new pads for received rtp packet
  g_signal_connect(session_->rtpsession_, "pad-added", G_CALLBACK(on_pad_added), this);
  // FIXME have this in rtpsession2: g_signal_connect(session_->rtpsession_,
  // "request-pt-map", (GCallback)request_pt_map, this);
  // configuring shmdatasrc and typefind
  g_signal_connect(typefind_, "have-type", G_CALLBACK(on_caps), this);
  g_object_set(
      G_OBJECT(shmdatasrc_), "socket-path", rtpshmpath_.c_str(), "copy-buffers", TRUE, nullptr);
  gst_bin_add_many(
      GST_BIN(session_->gst_pipeline_->get_pipeline()), shmdatasrc_, typefind_, nullptr);
  gst_element_link(shmdatasrc_, typefind_);
  GstUtils::sync_state_with_parent(shmdatasrc_);
  GstUtils::sync_state_with_parent(typefind_);
}

RTPReceiver::~RTPReceiver() {
  gst_element_unlink(shmdatasrc_, typefind_);
  GstUtils::clean_element(typefind_);
  gst_element_set_state(shmdatasrc_, GST_STATE_NULL);
  gst_bin_remove(GST_BIN(session_->gst_pipeline_->get_pipeline()), shmdatasrc_);
  if (rtp_sink_pad_) gst_element_release_request_pad(session_->rtpsession_, rtp_sink_pad_);
  if (shmsrc_caps_) gst_caps_unref(shmsrc_caps_);
}

void RTPReceiver::on_caps(GstElement* /*typefind*/,
                          guint /*probability */,
                          GstCaps* caps,
                          gpointer user_data) {
  RTPReceiver* context = static_cast<RTPReceiver*>(user_data);
  std::string rtp_id;
  context->shmsrc_caps_ = gst_caps_copy(caps);
  // link the payloader with the rtpbin
  {
    context->rtp_sink_pad_ =
        gst_element_get_request_pad(context->session_->rtpsession_, "recv_rtp_sink_%u");
    On_scope_exit { gst_object_unref(context->rtp_sink_pad_); };
    GstPad* srcpad = gst_element_get_static_pad(context->typefind_, "src");
    On_scope_exit { gst_object_unref(srcpad); };
    gst_pad_link(srcpad, context->rtp_sink_pad_);
    gchar* rtp_sink_pad_name = gst_pad_get_name(context->rtp_sink_pad_);
    On_scope_exit { g_free(rtp_sink_pad_name); };
    gchar** rtpsession_array = g_strsplit_set(rtp_sink_pad_name, "_", 0);
    On_scope_exit { g_strfreev(rtpsession_array); };
    rtp_id = std::string(rtpsession_array[3]);
    context->rtp_src_pad_prefix_ = std::string("recv_rtp_src_") + rtp_id + '_';
    // TODO send RTCP
  }
}

void RTPReceiver::on_pad_added(GstElement* /*object*/, GstPad* pad, gpointer user_data) {
  RTPReceiver* context = static_cast<RTPReceiver*>(user_data);
  gchar* name_cstr = gst_pad_get_name(pad);
  On_scope_exit { g_free(name_cstr); };
  auto name = std::string(name_cstr);
  auto& prefix = context->rtp_src_pad_prefix_;
  // testing if this is the right callback
  if (prefix.empty() || 0 != std::string(name).compare(0, prefix.size(), prefix)) {
    return;
  }
  if (!context->decodebin_.invoke_with_return<bool>([&](GstElement* el) {
        gst_bin_add(GST_BIN(context->session_->gst_pipeline_->get_pipeline()), el);
        GstPad* sinkpad = gst_element_get_static_pad(el, "sink");
        On_scope_exit { gst_object_unref(sinkpad); };
        gst_pad_link(pad, sinkpad);
        GstUtils::sync_state_with_parent(el);
        return true;
      })) {
  }
}

GstCaps* RTPReceiver::request_pt_map(GstElement* /*sess*/,
                                     guint /*session*/,
                                     guint /*pt*/,
                                     gpointer user_data) {
  RTPReceiver* context = static_cast<RTPReceiver*>(user_data);
  auto caps = context->shmsrc_caps_;
  context->shmsrc_caps_ = nullptr;
  return caps;
}

}  // namespace switcher
