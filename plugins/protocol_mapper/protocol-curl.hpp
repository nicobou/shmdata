/*
 * This file is part of switcher-protocol-mapper.
 *
 * switcher-curl is free software; you can redistribute it and/or
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
#ifndef SWITCHER_PROTOCOL_CURL_HPP
#define SWITCHER_PROTOCOL_CURL_HPP

#include <curl/curl.h>
#include "protocol-reader.hpp"
#include "switcher/quiddity/quiddity.hpp"

namespace switcher {

class ProtocolCurl : public ProtocolReader {
 public:
  ProtocolCurl(Quiddity* quid, const InfoTree* tree);
  ~ProtocolCurl();
  bool make_properties(Quiddity* quid, const InfoTree* tree) final;

 private:
  static constexpr const int kTimeout{3000};
  static std::atomic<int> instance_count_;
  CURL* curl_{nullptr};

  bool safe_bool_idiom() const final { return curl_ != nullptr; };
  bool curl_request(const std::string& url);
};
}

#endif
