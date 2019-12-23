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

#include "./rtp-session.hpp"

namespace switcher {

RtpSession::RtpSession()
    : gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      rtpsession_(GstUtils::make_element("rtpbin", nullptr)) {
  if (!gst_pipeline_ || !rtpsession_) {
    if (rtpsession_) gst_object_unref(rtpsession_);
    return;
  }
  g_object_set(G_OBJECT(rtpsession_),
               //"ntp-sync", TRUE,
               "async-handling",
               TRUE,
               "latency",
               70,
               "drop-on-latency",
               TRUE,
               "do-lost",
               TRUE,
               nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), rtpsession_);
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  gst_pipeline_->play(true);
}

}  // namespace switcher
