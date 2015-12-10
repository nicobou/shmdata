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

namespace switcher {

RtpSession2::RtpSession2():
    gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)),
    rtpsession_(GstUtils::make_element("rtpbin", nullptr)){
  if (!gst_pipeline_ || !rtpsession_){
    if (rtpsession_)
      gst_object_unref(rtpsession_);
    return;    
  }
  g_object_set(G_OBJECT(rtpsession_), "ntp-sync", TRUE, "async-handling", TRUE, nullptr);
  g_signal_connect(G_OBJECT(rtpsession_), "on-bye-ssrc",
                   (GCallback) on_bye_ssrc, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-bye-timeout",
                   (GCallback) on_bye_timeout, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-new-ssrc",
                   (GCallback) on_new_ssrc, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-npt-stop",
                   (GCallback) on_npt_stop, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-sender-timeout",
                   (GCallback) on_sender_timeout, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-active",
                   (GCallback) on_ssrc_active, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-collision",
                   (GCallback) on_ssrc_collision, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-sdes",
                   (GCallback) on_ssrc_sdes, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-validated",
                   (GCallback) on_ssrc_validated, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-timeout",
                   (GCallback) on_timeout, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "pad-added",
                   (GCallback) on_pad_added, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "pad-removed",
                   (GCallback) on_pad_removed, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "no-more-pads",
                   (GCallback) on_no_more_pad, (gpointer) this);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), rtpsession_);
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()),
               "async-handling", TRUE,
               nullptr);
  gst_pipeline_->play(true);
}


void RtpSession2::on_bye_ssrc(GstElement */*rtpbin */,
                             guint /*session */,
                             guint /*ssrc */,
                             gpointer /*user_data */) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_bye_ssrc");
}

void RtpSession2::on_bye_timeout(GstElement * /*rtpbin */ ,
                                guint /*session */ ,
                                guint /*ssrc */ ,
                                gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_bye_timeout");
}

void RtpSession2::on_new_ssrc(GstElement * /*rtpbin */ ,
                             guint /*session */ ,
                             guint /*ssrc */ ,
                             gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  //g_debug("on_new_ssrc");
}

void RtpSession2::on_npt_stop(GstElement * /*rtpbin */ ,
                             guint /*session */ ,
                             guint /*ssrc */ ,
                             gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_npt_stop");
}

void RtpSession2::on_sender_timeout(GstElement * /*rtpbin */ ,
                                   guint /*session */ ,
                                   guint /*ssrc */ ,
                                   gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_sender_timeout");
}

void RtpSession2::on_ssrc_active(GstElement * /*rtpbin */ ,
                                guint /*session */ ,
                                guint /*ssrc */ ,
                                gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_ssrc_active");
}

void RtpSession2::on_ssrc_collision(GstElement * /*rtpbin */ ,
                                   guint /*session */ ,
                                   guint /*ssrc */ ,
                                   gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  //g_debug("on_ssrc_active");
}

void RtpSession2::on_ssrc_sdes(GstElement * /*rtpbin */ ,
                              guint /*session */ ,
                              guint /*ssrc */ ,
                              gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_ssrc_sdes");
}

void RtpSession2::on_ssrc_validated(GstElement * /*rtpbin */ ,
                                   guint /*session */ ,
                                   guint /*ssrc */ ,
                                   gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_ssrc_validated");
}

void RtpSession2::on_timeout(GstElement * /*rtpbin */ ,
                            guint /*session */ ,
                            guint /*ssrc */ ,
                            gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_timeout");
}

void RtpSession2::on_pad_added(GstElement * /*gstelement */ ,
                               GstPad */*new_pad*/,
                               gpointer /*user_data*/) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_pad_added, name: %s, direction: %d",
  //         gst_pad_get_name(new_pad), gst_pad_get_direction(new_pad));
}

void RtpSession2::on_pad_removed(GstElement * /*gstelement */ ,
                                GstPad */*new_pad*/,
                                gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_pad_removed");
}

void RtpSession2::on_no_more_pad(GstElement * /*gstelement */ ,
                                gpointer /*user_data */ ) {
  // RtpSession2 *context = static_cast<RtpSession2 *>(user_data);
  // g_debug("on_no_more_pad");
}

}  // namespace switcher
