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

#include <pjsua-lib/pjsua.h>

namespace switcher {
namespace PJCallUtils {

bool is_receive_media(const pjmedia_sdp_media *media){
  return pjmedia_sdp_media_find_attr2(media, "recvonly", nullptr) != nullptr
      || pjmedia_sdp_media_find_attr2(media, "sendrecv", nullptr) != nullptr;
}

bool is_send_media(const pjmedia_sdp_media *media){
  return pjmedia_sdp_media_find_attr2(media, "sendonly", nullptr) != nullptr
      || pjmedia_sdp_media_find_attr2(media, "sendrecv", nullptr) != nullptr;
}

bool is_receiving(const pjmedia_sdp_session *sdp){
  for (uint i = 0; i < sdp->media_count; i++)
    if (PJCallUtils::is_receive_media(sdp->media[i]))
      return true;
  return false;
}

}  // namespace PJCallUtils
}  // namespace switcher
#endif
