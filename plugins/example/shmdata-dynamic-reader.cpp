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

#include "./shmdata-dynamic-reader.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataDynReader,
                                     "dyn-reader-quid",
                                     "Example Quiddity with Dynamic Reader Shmdata",
                                     "Dummy plugin for testing/example purpose",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string ShmdataDynReader::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "video%",
      "description": "Video streams",
      "can_do": ["video/x-raw"]
    }
  ]
}
)");

ShmdataDynReader::ShmdataDynReader(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t sfid) {
                  return on_connect(shmpath, sfid);
                },
                [this](claw::sfid_t sfid) { return on_disconnect(sfid); }}) {}

bool ShmdataDynReader::on_connect(const std::string& /*shmpath*/, claw::sfid_t /*sfid*/) {
  return true;
}

bool ShmdataDynReader::on_disconnect(claw::sfid_t /*sfid*/) { return true; }
}  // namespace quiddities
}  // namespace switcher
