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

#ifndef __SWITCHER_GST_VIDEO_ENCODER_H__
#define __SWITCHER_GST_VIDEO_ENCODER_H__

#include <memory>
#include "../gst/video-codec.hpp"
#include "../quiddity/quiddity.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class GstVideoEncoder : public Quiddity {
 public:
  GstVideoEncoder(quiddity::Config&&);

 private:
  static const std::string kConnectionSpec;  //!< Shmdata specifications
  std::unique_ptr<gst::VideoCodec> codecs_{nullptr};

  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string& shmdata_socket_path);
};

}  // namespace quiddities
}  // namespace switcher
#endif
