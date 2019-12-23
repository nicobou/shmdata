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

#ifndef __SWITCHER_RTP_SENDER_H__
#define __SWITCHER_RTP_SENDER_H__

#include <gst/gst.h>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <string>

namespace switcher {
class RtpSession;

class RTPSender {
 public:
  using id_t = size_t;
  using frame_cb_t = std::function<void(void*, size_t)>;
  RTPSender(RtpSession* session,
            const std::string& shmpath,
            unsigned int mtu  // = 1400
            );
  RTPSender() = delete;
  ~RTPSender();
  RTPSender(const RTPSender&) = delete;
  RTPSender(RTPSender&&) = delete;
  RTPSender& operator=(const RTPSender&) = delete;

  id_t add_cb(frame_cb_t fun);
  bool remove_cb(id_t cb_id);
  // get caps for data in the callback:
  std::string get_caps() const;

 private:
  RtpSession* session_;
  std::string shmpath_;
  unsigned int mtu_;
  GstElement* shmdatasrc_;
  GstElement* typefind_;
  GstElement* rtp_payloader_{nullptr};
  GstElement* fakesink_;
  GstPad* rtp_sink_pad_{nullptr};
  id_t counter_{0};
  // std::string rtp_id_{};
  std::map<id_t, frame_cb_t> rtp_frame_cbs_{};
  std::mutex mtx_{};
  mutable std::string fakesink_caps_{};
  std::mutex start_m_{};
  std::condition_variable start_cv_{};

  static void on_handoff_cb(GstElement* /*object*/,
                            GstBuffer* buf,
                            GstPad* pad,
                            gpointer user_data);
  static void on_caps(GstElement* typefind,
                      guint /*probability */,
                      GstCaps* caps,
                      gpointer /*user_data*/);
};

}  // namespace switcher
#endif
