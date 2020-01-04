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
#include "../infotree/information-tree-json.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ExternalWriter,
                                     "extshmsrc",
                                     "Raw Shmdata",
                                     "other",
                                     "writer",
                                     "Import an external shmdata writer",
                                     "LGPL",
                                     "Nicolas Bouillot");

ExternalWriter::ExternalWriter(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)) {
  pmanage<MPtr(&property::PBag::make_string)>(
      "shmdata-path",
      [this](const std::string& val) {
        shmdata_path_ = val;
        shm_ = std::make_unique<shmdata::Follower>(this,
                                                   shmdata_path_,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   shmdata::Stat::kDefaultUpdateInterval,
                                                   shmdata::Follower::Direction::writer);
        return true;
      },
      [this]() { return shmdata_path_; },
      "Shmdata Path",
      "Path Of The Shmdata The Include",
      "");
}

InfoTree::ptr ExternalWriter::on_saving() {
  return infotree::json::deserialize(tree<MPtr(&InfoTree::serialize_json)>(".shmdata.writer."));
}

void ExternalWriter::on_loading(InfoTree::ptr&& tree) {
  if (tree->empty()) return;
  graft_tree(".shmdata.writer.", tree);
}

}  // namespace quiddities
}  // namespace switcher
