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

#ifndef __SWITCHER_SHMDATA_DECODER_H__
#define __SWITCHER_SHMDATA_DECODER_H__

#include "./decodebin-to-shmdata.hpp"
#include "./gst-pipeliner.hpp"
#include "./gst-shmdata-subscriber.hpp"
#include "./quiddity.hpp"

namespace switcher {
class RtpSession2;

class ShmdataDecoder {
 public:
  using on_shmwriter_path_t = std::function<void(const std::string&)>;
  ShmdataDecoder(
      Quiddity* quid,
      GstPipeliner* pipeliner,
      const std::string& shmpath,
      const std::string& shm_prefix,   // if empty, use quid's prefix method
      const std::string& media_label,  // ignored if empty
      on_shmwriter_path_t cb);
  ShmdataDecoder() = delete;
  ~ShmdataDecoder();
  ShmdataDecoder(const ShmdataDecoder&) = delete;
  ShmdataDecoder(ShmdataDecoder&&) = delete;
  ShmdataDecoder& operator=(const ShmdataDecoder&) = delete;

 private:
  void configure_shmdatasink(GstElement* element,
                             const std::string& media_type,
                             const std::string& media_label);
  Quiddity* quid_;
  GstPipeliner* pipeliner_;
  std::string shmpath_;
  std::string shmwriter_path_{};
  std::string shm_prefix_;
  std::string media_label_;
  GstElement* shmdatasrc_;
  DecodebinToShmdata decodebin_;
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{};
  on_shmwriter_path_t on_shmwriter_path_cb_;
};

}  // namespace switcher
#endif
