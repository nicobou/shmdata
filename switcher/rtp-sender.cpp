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

#include "./rtp-sender.hpp"
#include "./rtp-session2.hpp"
#include "./scope-exit.hpp"
#include "switcher/gst-rtppayloader-finder.hpp"

namespace switcher {

RTPSender::RTPSender(RtpSession2* session, const std::string& shmpath, unsigned int mtu)
    : session_(session),
      shmpath_(shmpath),
      mtu_(mtu),
      shmdatasrc_(gst_element_factory_make("shmdatasrc", nullptr)),
      typefind_(gst_element_factory_make("typefind", nullptr)),
      fakesink_(gst_element_factory_make("fakesink", nullptr)) {
  std::unique_lock<std::mutex> lock(start_m_);
  // configuring shmdatasrc and typefind
  g_signal_connect(typefind_, "have-type", G_CALLBACK(on_caps), this);
  g_object_set(
      G_OBJECT(shmdatasrc_), "socket-path", shmpath_.c_str(), "copy-buffers", TRUE, nullptr);
  gst_bin_add_many(
      GST_BIN(session_->gst_pipeline_->get_pipeline()), shmdatasrc_, typefind_, nullptr);
  if (!gst_element_link(shmdatasrc_, typefind_)) return;
  // configuring fakesink
  g_object_set(
      G_OBJECT(fakesink_), "silent", TRUE, "signal-handoffs", TRUE, "sync", FALSE, nullptr);
  g_signal_connect(fakesink_, "handoff", (GCallback)on_handoff_cb, this);
  GstUtils::sync_state_with_parent(shmdatasrc_);
  GstUtils::sync_state_with_parent(typefind_);
  start_cv_.wait_for(lock, std::chrono::milliseconds(200));
}

RTPSender::~RTPSender() {
  if (shmdatasrc_) GstUtils::clean_element(shmdatasrc_);
  if (typefind_) GstUtils::clean_element(typefind_);
  if (rtp_payloader_) GstUtils::clean_element(rtp_payloader_);
  if (fakesink_) GstUtils::clean_element(fakesink_);
  if (rtp_sink_pad_) gst_element_release_request_pad(session_->rtpsession_, rtp_sink_pad_);
}

void RTPSender::on_caps(GstElement* typefind,
                        guint /*probability */,
                        GstCaps* caps,
                        gpointer user_data) {
  RTPSender* context = static_cast<RTPSender*>(user_data);
  // making the RTP payloader
  GstElementFactory* fact = GstRTPPayloaderFinder::get_factory_by_caps(caps);
  if (nullptr != fact)
    context->rtp_payloader_ = gst_element_factory_create(fact, nullptr);
  else
    context->rtp_payloader_ = gst_element_factory_make("rtpgstpay", nullptr);
  g_object_set(G_OBJECT(context->rtp_payloader_), "mtu", (guint)context->mtu_, nullptr);
  // g_object_set(G_OBJECT(context->rtp_payloader_), "mtu", 5000, nullptr);
  gst_bin_add_many(GST_BIN(context->session_->gst_pipeline_->get_pipeline()),
                   context->rtp_payloader_,
                   context->fakesink_,
                   nullptr);
  if (!gst_element_link(typefind, context->rtp_payloader_))
    g_error("BUG issue linking typefind with fakesink in RTPSender::on_caps");
  std::string rtp_id;
  // link the payloader with the rtpbin
  {
    context->rtp_sink_pad_ =
        gst_element_get_request_pad(context->session_->rtpsession_, "send_rtp_sink_%u");
    On_scope_exit { gst_object_unref(context->rtp_sink_pad_); };
    GstPad* srcpad = gst_element_get_static_pad(context->rtp_payloader_, "src");
    On_scope_exit { gst_object_unref(srcpad); };
    gst_pad_link(srcpad, context->rtp_sink_pad_);
    gchar* rtp_sink_pad_name = gst_pad_get_name(context->rtp_sink_pad_);
    On_scope_exit { g_free(rtp_sink_pad_name); };
    gchar** rtpsession_array = g_strsplit_set(rtp_sink_pad_name, "_", 0);
    On_scope_exit { g_strfreev(rtpsession_array); };
    rtp_id = std::string(rtpsession_array[3]);
    // TODO receive RTCP
  }
  // linking rtpbin to fakesink
  {
    GstPad* src_pad = gst_element_get_static_pad(context->session_->rtpsession_,
                                                 std::string("send_rtp_src_" + rtp_id).c_str());
    On_scope_exit { gst_object_unref(src_pad); };
    GstPad* sink_pad = gst_element_get_static_pad(context->fakesink_, "sink");
    On_scope_exit { gst_object_unref(sink_pad); };
    gst_pad_link(src_pad, sink_pad);
    // TODO send RTCP
  }

  // syncing gst elements with pipeline
  GstUtils::sync_state_with_parent(context->rtp_payloader_);
  GstUtils::sync_state_with_parent(context->fakesink_);
  std::unique_lock<std::mutex> lock(context->start_m_);
  context->start_cv_.notify_one();
}

void RTPSender::on_handoff_cb(GstElement* /*object*/,
                              GstBuffer* buf,
                              GstPad* /*pad*/,
                              gpointer user_data) {
  RTPSender* context = static_cast<RTPSender*>(user_data);
  // getting buffer information:
  GstMapInfo map;
  if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
    return;
  }
  On_scope_exit { gst_buffer_unmap(buf, &map); };
  std::unique_lock<std::mutex> lock(context->mtx_);
  for (auto& it : context->rtp_frame_cbs_) it.second(map.data, map.size);
}

std::string RTPSender::get_caps() const {
  if (!fakesink_caps_.empty()) return fakesink_caps_;
  if (nullptr == fakesink_) {
    return std::string();
  }
  GstPad* pad = gst_element_get_static_pad(fakesink_, "sink");
  // GstPad *pad = gst_element_get_static_pad(rtp_payloader_, "src");
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

RTPSender::id_t RTPSender::add_cb(frame_cb_t fun) {
  if (!fun) return 0;
  std::unique_lock<std::mutex> lock(mtx_);
  auto res = ++counter_;
  rtp_frame_cbs_[res] = fun;
  return res;
}

bool RTPSender::remove_cb(id_t cb_id) {
  std::unique_lock<std::mutex> lock(mtx_);
  auto it = rtp_frame_cbs_.find(cb_id);
  if (it == rtp_frame_cbs_.end()) return false;
  rtp_frame_cbs_.erase(it);
  return true;
}

}  // namespace switcher
