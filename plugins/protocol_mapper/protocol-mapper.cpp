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
#include "protocol-mapper.hpp"
#include "switcher/file-utils.hpp"
#include "switcher/information-tree-json.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ProtocolMapper,
                                     "protocol-mapper",
                                     "Protocol to property mapper",
                                     "controller",
                                     "hid/device",
                                     "Maps properties to a protocol",
                                     "LGPL",
                                     "Nicolas Bouillot/Jérémie Soria");

ProtocolMapper::ProtocolMapper(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)) {
  config_file_id_ = pmanage<MPtr(&PContainer::make_string)>(
      "config_file",
      [this](const std::string& val) {
        config_file_ = val;
        auto file_content = FileUtils::get_file_content(val);
        if (file_content.first.empty()) {
          message("ERROR: %", file_content.second);
          return false;
        }
        auto tree = JSONSerializer::deserialize(file_content.first);
        if (!tree) {
          message("ERROR: % is not a valid json file", val);
          return false;
        }
        protocol_reader_.reset();
        protocol_reader_ = ProtocolReader::create_protocol_reader(this, tree.get());

        if (!protocol_reader_) {
          warning("Could not create protocol reader (protocol-mapper).");
          return false;
        }

        // Create quiddity properties from "commands" section of the json.
        if (!protocol_reader_->make_properties(this, tree->get_tree("commands").get()))
          return false;

        pmanage<MPtr(&PContainer::disable)>(config_file_id_, "Service already loaded");

        return true;
      },
      [this]() { return config_file_; },
      "Protocol mapping description description file",
      "This property allows for loading the command description",
      config_file_);
}
}
