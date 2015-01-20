/*
 * This file is part of switcher-pjsip.
 *
 * switcher-pjsip is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SWITCHER_PJCODEC_UTILS_H__
#define __SWITCHER_PJCODEC_UTILS_H__

#include <gst/gst.h>
#include <vector>
#include <string>
#include <memory>

namespace switcher {
struct RTPCodec {
  typedef std::unique_ptr<RTPCodec> ptr;
  std::string encoding_name_;
  int payload_;
  std::string media_;
  int clock_rate_;

  RTPCodec():encoding_name_(), payload_(-1), media_(), clock_rate_(-1) {
  }
};

namespace PJCodecUtils {
typedef std::vector<RTPCodec::ptr> codecs;
typedef std::vector<RTPCodec::ptr>::iterator codec_it;

PJCodecUtils::codecs inspect_rtp_codecs();
PJCodecUtils::codecs
inspect_rtp_codec_from_gst_element_factory(GstElementFactory *factory);
PJCodecUtils::codecs inspect_rtp_codec_from_gst_caps(GstCaps *caps);
PJCodecUtils::codecs inspect_rtp_codec_from_gst_struct(GstStructure *
                                                       caps_struct);

std::vector<std::string>
get_string_values_from_gst_struct(GstStructure *caps_struct,
                                  std::string key);
std::vector<gint>
get_int_values_from_gst_struct(GstStructure *caps_struct,
                               std::string key);
}  // namespace PJCodecUtils
}  // namespace switcher
#endif
