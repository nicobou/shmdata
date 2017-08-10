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

#include "./rtp-destination.hpp"
#include <glib/gstdio.h>  // writing sdp file
#include <sstream>
#include "./information-tree.hpp"
#include "./rtp-session.hpp"
#include "./scope-exit.hpp"
#include "./sdp-utils.hpp"

namespace switcher {
RtpDestination::RtpDestination(RtpSession* session)
    : json_description_(std::make_shared<JSONBuilder>()), session_(session) {}

RtpDestination::~RtpDestination() {
  for (auto& it : files_) g_remove(it.c_str());
}

void RtpDestination::set_name(std::string name) {
  name_ = std::move(name);
  make_json_description();
}

void RtpDestination::set_host_name(std::string host_name) {
  host_name_ = std::move(host_name);
  make_json_description();
}

std::string RtpDestination::get_host_name() const { return host_name_; }

std::string RtpDestination::get_port(const std::string& shmdata_path) {
  auto it = source_streams_.find(shmdata_path);
  if (source_streams_.end() == it) return std::string();
  return it->second;
}

bool RtpDestination::add_stream(const std::string& shmdata_path, std::string port) {
  source_streams_[shmdata_path] = port;
  make_json_description();
  return true;
}

bool RtpDestination::has_shmdata(const std::string& shmdata_path) {
  return (source_streams_.end() != source_streams_.find(shmdata_path));
}

bool RtpDestination::remove_stream(const std::string& shmdata_stream_path) {
  auto it = source_streams_.find(shmdata_stream_path);
  if (source_streams_.end() == it) {
    session_->warning("RtpDestination: stream not found, cannot remove %", shmdata_stream_path);
    return false;
  }
  source_streams_.erase(it);
  make_json_description();
  return true;
}

std::string RtpDestination::get_sdp() {
  SDPDescription desc;

  for (auto& it : source_streams_) {
    std::string string_caps = session_->tree<MPtr(&InfoTree::branch_read_data<std::string>)>(
        std::string("rtp_caps." + it.first));
    GstCaps* caps = gst_caps_from_string(string_caps.c_str());
    On_scope_exit { gst_caps_unref(caps); };
    gint port = atoi(it.second.c_str());
    SDPMedia media;
    if (!media.set_media_info_from_caps(caps)) session_->warning("issue with sdp media info");
    media.set_port(port);
    if (!desc.add_media(media)) {
      session_->warning(
          "a media has not been added to the SDP description,"
          "returning empty description");
      return std::string();
    }
  }
  return desc.get_string();
}

void RtpDestination::make_json_description() {
  json_description_->reset();
  json_description_->begin_object();
  json_description_->add_string_member("name", name_.c_str());
  json_description_->add_string_member("host_name", host_name_.c_str());
  json_description_->set_member_name("data_streams");
  json_description_->begin_array();
  for (auto& it : source_streams_) {
    json_description_->begin_object();
    json_description_->add_string_member("path", it.first.c_str());
    json_description_->add_string_member("port", it.second.c_str());
    json_description_->end_object();
  }
  json_description_->end_array();
  json_description_->end_object();
}

JSONBuilder::Node RtpDestination::get_json_root_node() { return json_description_->get_root(); }

bool RtpDestination::write_to_file(std::string file_name) {
  std::string sdp = get_sdp();
  if (sdp.empty()) return false;
  GError* error = NULL;
  if (!g_file_set_contents(file_name.c_str(),
                           sdp.c_str(),
                           -1,  // no size, res is a null terminated string
                           &error)) {
    session_->warning("%", std::string(error->message));
    g_error_free(error);
    return false;
  }
  files_.push_front(std::move(file_name));
  return true;
}

std::vector<std::string> RtpDestination::get_shmdata() const {
  std::vector<std::string> res;
  for (auto& it : source_streams_) {
    res.push_back(it.first);
  }
  return res;
}

}  // namespace switcher
