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
#ifndef SWITCHER_PROTOCOL_MAPPER_HPP
#define SWITCHER_PROTOCOL_MAPPER_HPP

#include "protocol-reader.hpp"
#include "switcher/quiddity.hpp"

namespace switcher {
class ProtocolMapper : public Quiddity {
 public:
  ProtocolMapper(QuiddityConfiguration&&);
  ~ProtocolMapper() = default;

 private:

  // Protocol configuration
  std::string config_file_{};
  PContainer::prop_id_t config_file_id_;
  std::unique_ptr<ProtocolReader> protocol_reader_{};
};

SWITCHER_DECLARE_PLUGIN(ProtocolMapper);
}

#endif
