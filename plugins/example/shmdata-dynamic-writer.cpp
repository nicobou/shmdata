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

#include "./shmdata-dynamic-writer.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataDynWriter,
                                     "dyn-writer-quid",
                                     "Example Quiddity with Dynamic Writer Shmdata",
                                     "test",
                                     "",
                                     "Dummy plugin for testing/example purpose",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string ShmdataDynWriter::kConnectionSpec(R"(
{
"writer":
  [
    {
      "label": "video%",
      "description": "Video streams",
      "can_do": ["video/x-raw"]
    }
  ]
}
)");

ShmdataDynWriter::ShmdataDynWriter(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf), {kConnectionSpec}),
      video_prop_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "video",
          [this](bool val) {
            if (val) {
              videoA_swid_ = claw_.add_writer_to_meta(claw_.get_swid("video%"),
                                                      {"videoA", "A is an video Shmdata"});
              videoB_swid_ = claw_.add_writer_to_meta(claw_.get_swid("video%"),
                                                      {"videoB", "B is an video Shmdata"});
            } else {
              claw_.remove_writer_from_meta(videoA_swid_);
              claw_.remove_writer_from_meta(videoB_swid_);
            }
            video_ = val;
            return true;
          },
          [this]() { return video_; },
          "Enable VideoA and VideoB Shmdata",
          "VideoA and VideoB are Shmdata related to the meta Shmdata labeled video%",
          video_)) {}

}  // namespace quiddities
}  // namespace switcher
