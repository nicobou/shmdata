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

#ifndef __SWITCHER_RTP_RECEIVER_H__
#define __SWITCHER_RTP_RECEIVER_H__

#include <gst/gst.h>
#include <functional>
#include <map>
#include <mutex>
#include "decodebin-to-shmdata.hpp"

namespace switcher {
class RtpSession2;

class RTPReceiver {
 public:
  using id_t = size_t;
  using configure_shmsink_cb_t = std::function<void(
      GstElement* el, const std::string& media_type, const std::string& media_label)>;
  RTPReceiver(RtpSession2* session,
              const std::string& rtpshmpath,
              configure_shmsink_cb_t cb,
              bool decompress);
  RTPReceiver() = delete;
  ~RTPReceiver();
  RTPReceiver(const RTPReceiver&) = delete;
  RTPReceiver(RTPReceiver&&) = delete;
  RTPReceiver& operator=(const RTPReceiver&) = delete;

 private:
  RtpSession2* session_;
  std::string rtpshmpath_;
  GstElement* shmdatasrc_;
  GstElement* typefind_;
  bool decompress_;
  configure_shmsink_cb_t configure_shmsink_cb_;
  DecodebinToShmdata decodebin_;
  GstCaps* shmsrc_caps_{nullptr};
  GstPad* rtp_sink_pad_{nullptr};  // check if this needs to be a member
  std::string rtp_src_pad_prefix_{};

  static void on_caps(GstElement* typefind, guint /*prob*/, GstCaps* caps, gpointer data);
  static void on_pad_added(GstElement* object, GstPad* pad, gpointer user_data);
  static GstCaps* request_pt_map(GstElement* sess, guint session, guint pt, gpointer user_data);
};

}  // namespace switcher
#endif
