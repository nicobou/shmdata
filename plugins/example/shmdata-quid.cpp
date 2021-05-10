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
      "name": "texture",
      "description": "Texture to apply during the processing",
      "caps": ["video/x-raw"]
    },
    {
      "name": "mic%",
      "description": "Audio inputs to be analysed",
      "caps": ["audio/x-raw", "audio/mpeg"]
    },
    {
      "name": "custom",
      "description": "Custom shmdata for my statistical analysis",
      "caps": []
    }
  ],
"writer":
  [
    {
      "name": "texture",
      "description": "Produced rendering",
      "caps": ["video/x-raw"]
    },
    {
      "name": "custom%",
      "description": "Produced random buffers",
      "caps": []
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
