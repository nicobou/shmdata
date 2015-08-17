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

#include <algorithm>
#include "./shmdata-utils.hpp"
#include "./gst-shmdata-subscriber.hpp"

namespace switcher {

std::string ShmdataUtils::get_category(const std::string &caps){
  std::string category;
  std::string mime_type(caps.begin(),
                        std::find(caps.begin(), caps.end(), (',')));
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
  } else if (std::string::npos !=
             mime_type.find("application/x-libloserialized-osc")) {
    category = "osc";
  } else if (std::string::npos != mime_type.find("application/x-")) {
    auto it = std::find(mime_type.begin(),
                        mime_type.end(),
                        '-');
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

data::Tree::ptr ShmdataUtils::make_tree(const std::string &caps,
                                        const std::string &category,
                                        GstShmdataSubscriber::num_bytes_t num_bytes){
  data::Tree::ptr tree = data::Tree::make();
  tree->graft(".caps", data::Tree::make(caps));
  tree->graft(".category", data::Tree::make(category));
  tree->graft(".byte_rate", data::Tree::make(num_bytes));
  return tree;
}


}  // namespace switcher
