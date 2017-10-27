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

#ifndef PLUGINS_PJSIP_PJ_CALL_UTILS_H_
#define PLUGINS_PJSIP_PJ_CALL_UTILS_H_

#include <glib.h>
#include <pjsua-lib/pjsua.h>
#include <regex>
#include "switcher/string-utils.hpp"

namespace switcher {
namespace PJCallUtils {

bool is_receive_media(const pjmedia_sdp_media* media) {
  return pjmedia_sdp_media_find_attr2(media, "recvonly", nullptr) != nullptr ||
         pjmedia_sdp_media_find_attr2(media, "sendrecv", nullptr) != nullptr;
}

bool is_send_media(const pjmedia_sdp_media* media) {
  return pjmedia_sdp_media_find_attr2(media, "sendonly", nullptr) != nullptr ||
         pjmedia_sdp_media_find_attr2(media, "sendrecv", nullptr) != nullptr;
}

bool is_receiving(const pjmedia_sdp_session* sdp) {
  for (uint i = 0; i < sdp->media_count; i++)
    if (PJCallUtils::is_receive_media(sdp->media[i])) return true;
  return false;
}

std::string get_media_label(const pjmedia_sdp_media* media) {
  std::string res = std::string("media-label");
  // for (unsigned i=0; i < media->desc.fmt_count; i++){
  //   g_print("fmt ------------ %.*s\n",
  //           (int)media->desc.fmt[i].slen,
  //           media->desc.fmt[i].ptr);
  // }
  // for (unsigned i=0; i < media->attr_count; i++){
  //   g_print("attr------------name %.*s value %.*s\n",
  //           (int)media->attr[i]->name.slen,
  //           media->attr[i]->name.ptr,
  //           (int)media->attr[i]->value.slen,
  //           media->attr[i]->value.ptr);
  // }
  auto name = std::string("media-label=");
  for (unsigned i = 0; i < media->attr_count; i++) {
    auto value = std::string(media->attr[i]->value.ptr, 0, media->attr[i]->value.slen);
    auto pos = value.find(name);
    if (std::string::npos != pos) {
      auto index = pos + name.size();
      return StringUtils::base64_decode(std::string(value, index, value.find(';', index)));
    }
  }
  return res;
}

std::string get_rtp_caps(const pjmedia_sdp_media* media) {
  if (std::string(media->desc.transport.ptr, 0, media->desc.transport.slen) != "RTP/AVP") {
    return std::string();
  }
#ifdef DEBUG
  if (media->desc.fmt_count > 1) std::cerr << "sdp media with several format (unhandled)" << '\n';
#endif
  std::string res = std::string("application/x-rtp");
  std::string clock_rate;
  std::string encoding_name;
  std::string more;
  for (unsigned i = 0; i < media->attr_count; i++) {
    if (std::string(media->attr[i]->name.ptr, 0, media->attr[i]->name.slen) == "rtpmap") {
      auto value = std::string(media->attr[i]->value.ptr, 0, media->attr[i]->value.slen);
      auto index = value.find(' ');
      auto sep = value.find('/');
      encoding_name = std::string(", encoding-name=(string)") +
                      std::string(value, index + 1, sep - (index + 1));
      auto cr_end = value.find('/', sep + 1);
      clock_rate =
          std::string(", clock-rate=(int)") +
          std::string(value, sep + 1, cr_end == std::string::npos ? cr_end : cr_end - (sep + 1));
    }
    if (std::string(media->attr[i]->name.ptr, 0, media->attr[i]->name.slen) == "fmtp") {
      auto value = std::string(media->attr[i]->value.ptr, 0, media->attr[i]->value.slen);
      auto index = value.find(' ');
      more = std::string(", ") + std::string(value, index + 1, std::string::npos);
      // transforming caps=... into caps=(string)""
      std::regex e("\\b(caps=)([^;]*)");
      more = std::regex_replace(more, e, "$1(string)\"$2\"");
      more = StringUtils::replace_string(more, "==\";", "\\==\";");
      more = StringUtils::replace_string(more, "=\";", "\\=\";");
      more = StringUtils::replace_char(more, ';', ", ");
    }
  }
  res += std::string(", media=(string)") +
         std::string(media->desc.media.ptr, 0, media->desc.media.slen) + clock_rate +
         encoding_name + more;
  return res;
}

}  // namespace PJCallUtils
}  // namespace switcher
#endif
