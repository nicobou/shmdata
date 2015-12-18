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

#include "./quiddity.hpp"
#include "./gst-pipeliner.hpp"
#include "./decodebin-to-shmdata.hpp"
#include "./gst-shmdata-subscriber.hpp"

namespace switcher {
class RtpSession2;

class ShmdataDecoder {
 public:
  ShmdataDecoder(Quiddity *quid, GstPipeliner *pipeliner, const std::string &shmpath);
  ShmdataDecoder() = delete;
  ~ShmdataDecoder();
  ShmdataDecoder(const ShmdataDecoder &) = delete;
  ShmdataDecoder(ShmdataDecoder &&) = delete;
  ShmdataDecoder &operator=(const ShmdataDecoder &) = delete;

 private:
  void configure_shmdatasink(GstElement *element,
                             const std::string &media_type,
                             const std::string &media_label);
  Quiddity *quid_;
  GstPipeliner *pipeliner_;
  std::string shmpath_;
  GstElement *shmdatasrc_;
  DecodebinToShmdata decodebin_;
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{};
};

}  // namespace switcher
#endif
