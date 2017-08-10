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

#include "./external-shmdata-writer.hpp"
#include "./information-tree-json.hpp"
#include "./shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ExternalShmdataWriter,
                                     "extshmsrc",
                                     "Raw Shmdata",
                                     "other",
                                     "writer",
                                     "import an external shmdata writer",
                                     "LGPL",
                                     "Nicolas Bouillot");

ExternalShmdataWriter::ExternalShmdataWriter(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)) {
  pmanage<MPtr(&PContainer::make_string)>(
      "shmdata-path",
      [this](const std::string& val) {
        shmdata_path_ = val;
        shm_ = std::make_unique<ShmdataFollower>(this,
                                                 shmdata_path_,
                                                 nullptr,
                                                 nullptr,
                                                 nullptr,
                                                 ShmdataStat::kDefaultUpdateInterval,
                                                 ".shmdata.writer.");
        return true;
      },
      [this]() { return shmdata_path_; },
      "Shmdata Path",
      "Path Of The Shmdata The Include",
      "");
}

InfoTree::ptr ExternalShmdataWriter::on_saving() {
  return JSONSerializer::deserialize(tree<MPtr(&InfoTree::serialize_json)>(".shmdata.writer."));
}

void ExternalShmdataWriter::on_loading(InfoTree::ptr&& tree) {
  if (tree->empty()) return;
  graft_tree(".shmdata.writer.", tree);
}

}  // namespace switcher
