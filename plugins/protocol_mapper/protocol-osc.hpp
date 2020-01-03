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
#ifndef SWITCHER_PROTOCOL_OSC_HPP
#define SWITCHER_PROTOCOL_OSC_HPP

#include <lo/lo_types.h>
#include "protocol-reader.hpp"

namespace switcher {
namespace quiddities {

class ProtocolOsc : public ProtocolReader {
 public:
  ProtocolOsc(Quiddity* quid, const InfoTree* tree);
  ~ProtocolOsc() = default;
  bool make_properties(Quiddity* quid, const InfoTree* tree) final;

 private:
  struct OscAddress {
    OscAddress(const std::string&);
    OscAddress() = delete;
    ~OscAddress();
    lo_address address{nullptr};
  };

  std::string url_{};
  std::unique_ptr<OscAddress> osc_address_;
  property::prop_id_t url_id_{0};

  bool safe_bool_idiom() const final { return true; };
  int send_osc_code(const std::string& type, const std::string& path, const Any& value);
};

}  // namespace quiddities
}  // namespace switcher
#endif
