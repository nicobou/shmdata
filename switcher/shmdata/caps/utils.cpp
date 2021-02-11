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

#include "./utils.hpp"

#include <algorithm>
#include <shmdata/type.hpp>

namespace switcher {
namespace shmdata {
namespace caps {

std::string get_category(const std::string& caps) {
  std::string category;
  std::string mime_type(caps.begin(), std::find(caps.begin(), caps.end(), (',')));
  if (std::string::npos != mime_type.find("video/x-raw")) {
    category = "video";
  } else if (std::string::npos != mime_type.find("video/x-")) {
    category = "compressed video";
  } else if (std::string::npos != mime_type.find("audio/midi")) {
    category = "midi";
  } else if (std::string::npos != mime_type.find("audio/x-raw")) {
    category = "audio";
  } else if (std::string::npos != mime_type.find("audio/x-")) {
    category = "compressed audio";
  } else if (std::string::npos != mime_type.find("application/x-libloserialized-osc")) {
    category = "osc";
  } else if (std::string::npos != mime_type.find("application/x-")) {
    auto it = std::find(mime_type.begin(), mime_type.end(), '-');
    it++;
    if (mime_type.end() == it)
      category = "unknown";
    else
      category = std::string(it, mime_type.end());
  } else {
    category = mime_type;
  }
  return category;
}

switcher::quiddity::qid_t get_quiddity_id(const std::string& caps) {
  auto id = ::shmdata::Type(caps).get("quiddity-id");
  if (!id.has_value()) return 0;
  return std::any_cast<int>(id);
}

std::string get_switcher_name(const std::string& caps) {
  auto caps_type = ::shmdata::Type(caps);
  return std::any_cast<std::string>(caps_type.get("switcher-name"));
}

}  // namespace caps
}  // namespace shmdata
}  // namespace switcher
