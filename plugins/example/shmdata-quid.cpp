/*
 * This file is part of switcher-plugin-example.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#include "./shmdata-quid.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataQuid,
                                     "connection-quid",
                                     "Example Quiddity with Shmdata connections",
                                     "test",
                                     "",
                                     "Dummy plugin for testing/example purpose",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string ShmdataQuid::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "texture",
      "description": "Texture to apply during the processing",
      "can_do": ["video/x-raw"]
    },
    {
      "label": "mic%",
      "description": "Audio inputs to be analysed",
      "can_do": ["audio/x-raw", "audio/mpeg"]
    },
    {
      "label": "custom",
      "description": "Custom shmdata for my statistical analysis",
      "can_do": [ "all" ]
    }
  ],
"writer":
  [
    {
      "label": "texture",
      "description": "Produced rendering",
      "can_do": ["video/x-raw"]
    },
    {
      "label": "custom%",
      "description": "Produced random buffers",
      "can_do": [ "all" ]
    }
  ]
}
)");

ShmdataQuid::ShmdataQuid(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t sfid) {
                  return on_connect(shmpath, sfid);
                },
                [this](claw::sfid_t sfid) { return on_disconnect(sfid); }}) {}

bool ShmdataQuid::on_connect(const std::string& /*shmpath*/, claw::sfid_t /*sfid*/) { return true; }

bool ShmdataQuid::on_disconnect(claw::sfid_t /*sfid*/) { return true; }

}  // namespace quiddities
}  // namespace switcher
