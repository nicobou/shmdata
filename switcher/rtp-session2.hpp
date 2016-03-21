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

#ifndef __SWITCHER_RTPSESSION2_H__
#define __SWITCHER_RTPSESSION2_H__

#include <gst/gst.h>
#include <memory>
#include "./gst-pipeliner.hpp"
#include "./std2.hpp"
#include "./safe-bool-idiom.hpp"
#include "./rtp-sender.hpp"
#include "./rtp-receiver.hpp"
#include "./shmdata-decoder.hpp"

namespace switcher {
class RtpSession2: public SafeBoolIdiom {
  friend RTPSender;
  friend RTPReceiver;
 public:
  RtpSession2();
  ~RtpSession2() = default;
  RtpSession2(const RtpSession2 &) = delete;
  RtpSession2(RtpSession2 &&) = delete;
  RtpSession2 &operator=(const RtpSession2 &) = delete;

 private:
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  GstElement *rtpsession_;

  // safe bool idiom
  bool safe_bool_idiom() const final {
    return nullptr != rtpsession_ &&  gst_pipeline_;
  }
  
  // internal rtpbin signals
  static void on_bye_ssrc(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_bye_timeout(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_new_ssrc(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_npt_stop(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_sender_timeout(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_ssrc_active(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_ssrc_collision(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_ssrc_sdes(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_ssrc_validated(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_timeout(GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);
  static void on_pad_added(GstElement *gstelement, GstPad *new_pad, gpointer user_data);
  static void on_pad_removed(GstElement *gstelement, GstPad *new_pad, gpointer user_data);
  static void on_no_more_pad(GstElement *gstelement, gpointer user_data);
};

}  // namespace switcher
#endif
