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

#include "./rtp-session2.hpp"
#include "./rtp-sender.hpp"
#include "./scope-exit.hpp"
#include "switcher/gst-rtppayloader-finder.hpp"

namespace switcher {

RTPSender::RTPSender(RtpSession2 *session,
                     const std::string &shmpath,
                     unsigned int rtp_id,
                     unsigned int mtu):
    session_(session),
    shmpath_(shmpath),
    rtp_id_(rtp_id),
    mtu_(mtu),
    shmdatasrc_(gst_element_factory_make("shmdatasrc", nullptr)),
    typefind_(gst_element_factory_make("typefind", nullptr)),
    fakesink_(gst_element_factory_make("fakesink", nullptr)) {
  if (nullptr == shmdatasrc_ || nullptr == typefind_ || nullptr == fakesink_){
    g_warning("RTPSender failled to create GStreamer element");
    return;
  }
  // configuring shmdatasrc and typefind
  g_signal_connect(typefind_, "have-type", G_CALLBACK(on_caps), this);
  g_object_set(G_OBJECT(shmdatasrc_),
               "socket-path", shmpath_.c_str(),
               nullptr);
  gst_bin_add_many(GST_BIN(session_->gst_pipeline_->get_pipeline()),
                   shmdatasrc_, typefind_, nullptr);
  if (!gst_element_link(shmdatasrc_, typefind_))
    return;
  GstUtils::sync_state_with_parent(shmdatasrc_);
  GstUtils::sync_state_with_parent(typefind_);
  // configuring fakesink
  g_object_set(G_OBJECT(fakesink_),
               "silent", TRUE,
               "signal-handoffs", TRUE,
               "sync", FALSE,
               nullptr);
  g_signal_connect(fakesink_, "handoff", (GCallback)on_handoff_cb, this);
}

RTPSender::~RTPSender(){
}

void RTPSender::on_caps(GstElement *typefind,
                        guint /*probability */ ,
                        GstCaps *caps,
                        gpointer user_data){
  RTPSender *context = static_cast<RTPSender *>(user_data);
  GstElementFactory *fact = GstRTPPayloaderFinder::get_factory_by_caps(caps);
  if (nullptr != fact)
    context->rtp_payloader_ = gst_element_factory_create(fact, nullptr);
  else
     context->rtp_payloader_ = gst_element_factory_make("rtpgstpay", nullptr);
  gst_bin_add_many(GST_BIN(context->session_->gst_pipeline_->get_pipeline()),
                   context->rtp_payloader_,
                   context->fakesink_,
                   nullptr);
  if (!gst_element_link_many(typefind,
                             context->rtp_payloader_,
                             context->fakesink_,
                             nullptr))
    g_error("BUG issue linking typefind with fakesink in RTPSender::on_caps");
  GstUtils::sync_state_with_parent(context->rtp_payloader_);
  GstUtils::sync_state_with_parent(context->fakesink_);
}

void RTPSender::on_handoff_cb(
    GstElement */*object*/,
    GstBuffer *buf,
    GstPad */*pad*/,
    gpointer user_data) {
  RTPSender *context = static_cast<RTPSender *>(user_data);
  //getting buffer information:
  GstMapInfo map;
  if (!gst_buffer_map (buf, &map, GST_MAP_READ)) {
    g_warning("gst_buffer_map failled: canceling audio buffer access");
    return;
  }
  On_scope_exit{gst_buffer_unmap (buf, &map);};
  std::unique_lock<std::mutex> lock(context->mtx_);
  for (auto &it: context->rtp_frame_cbs_)
    it.second (map.data, map.size);
}

std::string RTPSender::get_caps() const{
  if (!fakesink_caps_.empty())
    return fakesink_caps_;
  if (nullptr == fakesink_){
    g_warning("RTPSender::get_caps, source data type not known"
              ", returning empty string");
    return std::string();
  }
  GstPad *pad = gst_element_get_static_pad(fakesink_, "sink");
  if (nullptr == pad) return fakesink_caps_;
  On_scope_exit{gst_object_unref(pad);};
  GstCaps *caps = gst_pad_get_current_caps(pad);
  if (nullptr == caps) return fakesink_caps_;
  On_scope_exit{gst_caps_unref(caps);};
  gchar *str = gst_caps_to_string(caps);
  if (nullptr == str) return fakesink_caps_;
  On_scope_exit{g_free(str);};
  fakesink_caps_ = std::string(str);
  if (fakesink_caps_.empty())
    g_warning("RTPSender::get_caps, caps not available returning empty string");
  //g_print("-------------------- %s\n", fakesink_caps_.c_str());
  return fakesink_caps_;
}

RTPSender::id_t RTPSender::add_cb(frame_cb_t fun){
  if (!fun) return 0;
  std::unique_lock<std::mutex> lock(mtx_);
  auto res = ++counter_;
  rtp_frame_cbs_[res] = fun;
  return res;
}

bool RTPSender::remove_cb(id_t cb_id){
  std::unique_lock<std::mutex> lock(mtx_);
  auto it = rtp_frame_cbs_.find(cb_id);
  if (it == rtp_frame_cbs_.end())
    return false;
  rtp_frame_cbs_.erase(it);
  return true;
}

}  // namespace switcher
