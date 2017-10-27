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

#include "./sdp-utils.hpp"

namespace switcher {
SDPMedia::SDPMedia() { gst_sdp_media_new(&media_); }

SDPMedia::~SDPMedia() {
  gst_sdp_media_free(media_);
  if (nullptr != caps_structure_) gst_structure_free(caps_structure_);
}

BoolLog SDPMedia::set_media_info_from_caps(const GstCaps* media_caps) {
  if (nullptr == media_caps) {
    return BoolLog(false, "media description is missing when generating SDP media");
  }
  GstStructure* s = gst_caps_get_structure(media_caps, 0);
  if (nullptr == s) {
    return BoolLog(false, "cannot get structure from caps when generating SDP media");
  }
  gint value;
  if (nullptr == gst_structure_get_string(s, "media") ||
      nullptr == gst_structure_get_string(s, "encoding-name") ||
      !gst_structure_get_int(s, "payload", &value) ||
      !gst_structure_get_int(s, "clock-rate", &value)) {
    return BoolLog(false, "invalide media caps for SDP media");
  }
  caps_structure_ = gst_structure_copy(s);
  return BoolLog(true);
}

bool SDPMedia::set_port(uint port) {
  port_ = port;
  return true;
}

BoolLog SDPMedia::add_to_sdp_description(GstSDPMessage* sdp_description,
                                         uint index,
                                         const std::string& ip_addr) const {
  if (0 == port_ || nullptr == caps_structure_) {
    return BoolLog(false, "missing information for adding media to sdp description");
  }

  /* get media type and payload for the m= line */
  std::string caps_str(gst_structure_get_string(caps_structure_, "media"));
  gst_sdp_media_set_media(media_, caps_str.c_str());

  gint caps_pt = 0;
  gst_structure_get_int(caps_structure_, "payload", &caps_pt);
  gst_sdp_media_add_format(media_, std::to_string(caps_pt).c_str());

  gst_sdp_media_set_port_info(media_, port_, 1);
  gst_sdp_media_set_proto(media_, "RTP/AVP");

  /* for the c= line */
  gst_sdp_media_add_connection(media_, "IN", "IP4", ip_addr.c_str(), 16, 0);

  // sendonly
  gst_sdp_media_add_attribute(media_, "sendonly", nullptr);

  /* get clock-rate, media type and params for the rtpmap attribute */
  gint caps_rate = 0;
  gst_structure_get_int(caps_structure_, "clock-rate", &caps_rate);
  std::string caps_enc(gst_structure_get_string(caps_structure_, "encoding-name"));
  std::string rtpmap(std::to_string(caps_pt) + " " + caps_enc + "/" + std::to_string(caps_rate));

  const gchar* caps_params = gst_structure_get_string(caps_structure_, "encoding-params");
  if (nullptr != caps_params) {
    rtpmap.append("/");
    rtpmap.append(caps_params);
  }
  gst_sdp_media_add_attribute(media_, "rtpmap", rtpmap.c_str());
  /* the config uri */
  std::string control("stream=" + std::to_string(index));
  gst_sdp_media_add_attribute(media_, "control", control.c_str());
  /* collect all other properties and add them to fmtp */
  std::string fmtp = std::to_string(caps_pt);
  fmtp.append(" ");
  bool first = true;
  guint n_fields = gst_structure_n_fields(caps_structure_);
  for (uint j = 0; j < n_fields; j++) {
    const gchar* fname_c = gst_structure_nth_field_name(caps_structure_, j);
    if (nullptr == fname_c) continue;
    std::string fname(fname_c);
    /* filter out standard properties */
    if (fname.compare("media") == 0 || fname.compare("payload") == 0 ||
        fname.compare("clock-rate") == 0 || fname.compare("encoding-name") == 0 ||
        fname.compare("encoding-params") == 0 || fname.compare("ssrc") == 0 ||
        fname.compare("clock-base") == 0 || fname.compare("seqnum-base") == 0)
      continue;
    const gchar* struct_str = gst_structure_get_string(caps_structure_, fname.c_str());
    if (nullptr == struct_str) continue;
    std::string val = std::string(struct_str);
    if (0 == fname.compare("sprop-parameter-sets")) {
      auto equal_pos = val.find('=');
      if (std::string::npos != equal_pos) {
        // removing buggy trailing = at the end of sprop-parameter-sets
        val = std::string(val, 0, equal_pos);
      }
      auto comma_pos = val.find(',');
      if (std::string::npos != comma_pos) {
        // removing buggy trailing comma and the rest from sprop-parameter-sets
        val = std::string(val, 0, comma_pos);
      }
    }
    std::string fname_value(fname + "=" + val);
    if (gst_structure_get_string(caps_structure_, fname.c_str())) {
      if (!first)
        fmtp.append(";");
      else
        first = false;
      fmtp.append(fname_value);
    }
  }
  if (!first) gst_sdp_media_add_attribute(media_, "fmtp", fmtp.c_str());
  for (auto& it : ice_candidate_values_) {
    gst_sdp_media_add_attribute(media_, "candidate", it.c_str());
  }
  gst_sdp_message_add_media(sdp_description, media_);
  return BoolLog(true);
}

bool SDPMedia::add_ice_candidate(const std::string& candidate_value) {
  ice_candidate_values_.push_back(candidate_value);
  return true;
}

SDPDescription::SDPDescription() : SDPDescription("127.0.0.1") {}

SDPDescription::SDPDescription(const std::string& ip_addr) : ip_addr_(ip_addr) {
  gst_sdp_message_new(&sdp_description_);
  /* some standard things first */
  gst_sdp_message_set_version(sdp_description_, "0");
  // FIXME check and chose between IP4 and IP6, IP4 hardcoded
  // FIXME generate proper session id &version
  gst_sdp_message_set_origin(sdp_description_,
                             "-",                 // the user name
                             "1188340656180883",  // a session id
                             "1",                 // a session version
                             "IN",                // a network type
                             "IP4",               // an address type
                             ip_addr.c_str());    // an address
  gst_sdp_message_set_session_name(sdp_description_, "switcher");
  gst_sdp_message_set_information(sdp_description_, "telepresence");
  gst_sdp_message_add_time(sdp_description_, "0", "0", nullptr);
  gst_sdp_message_add_attribute(sdp_description_, "tool", "switcher");
  gst_sdp_message_add_attribute(sdp_description_, "type", "broadcast");
  gst_sdp_message_add_attribute(sdp_description_, "control", "*");
}

SDPDescription::~SDPDescription() { gst_sdp_message_free(sdp_description_); }

bool SDPDescription::add_msg_attribute(const std::string& name, const std::string& value) {
  return GST_SDP_OK == gst_sdp_message_add_attribute(sdp_description_, name.c_str(), value.c_str());
}

std::string SDPDescription::get_string() {
  gchar* tmp = gst_sdp_message_as_text(sdp_description_);
  std::string res(tmp);
  g_free(tmp);
  return res;
}

bool SDPDescription::add_media(const SDPMedia& media) {
  if (!media.add_to_sdp_description(sdp_description_, index_, ip_addr_)) return false;
  index_++;
  return true;
}

}  // namespace switcher
