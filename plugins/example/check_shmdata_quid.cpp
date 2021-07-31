/*
 * This file is part of switcher-plugin-example.
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

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <map>

#include "switcher/infotree/information-tree.hpp"
#include "switcher/infotree/json-serializer.hpp"
#include "switcher/quiddity/basic-test.hpp"

int main() {
  {
    using namespace switcher;
    using namespace quiddity;
    using namespace claw;

    Switcher::ptr swit = Switcher::make_switcher("test_manager");
    assert(quiddity::test::full(swit, "connection-quid"));

    auto reader = swit->quids<MPtr(&Container::create)>("connection-quid", "reader", nullptr).get();
    assert(reader);

    auto writer = swit->quids<MPtr(&Container::create)>("connection-quid", "writer", nullptr).get();
    assert(writer);

    // get the writer connection information from the Quiddity conspec tree
    std::map<std::string, swid_t> swids;
    {
      auto conspec_tree = writer->conspec<MPtr(&InfoTree::get_copy)>();
      assert(conspec_tree);
      assert(!conspec_tree->empty());
      conspec_tree->cfor_each_in_array("writer", [&](const InfoTree* tree) {
        swids.emplace(tree->branch_get_value("label"), tree->branch_get_value("swid"));
      });
      assert(!swids.empty());
    }

    // get the reader connection information from the Quiddity conspec tree
    std::map<std::string, sfid_t> sfids;
    {
      auto conspec_tree = reader->conspec<MPtr(&InfoTree::get_copy)>();
      assert(conspec_tree);
      assert(!conspec_tree->empty());
      conspec_tree->cfor_each_in_array("follower", [&](const InfoTree* tree) {
        sfids.emplace(tree->branch_get_value("label"), tree->branch_get_value("sfid"));
      });
      assert(!sfids.empty());
    }

    {  // check compatibility of shmdata before trying to connect
      const auto wtexture_id = writer->claw<MPtr(&Claw::get_swid)>("texture");
      const auto wtypes = writer->claw<MPtr(&Claw::get_writer_can_do)>(wtexture_id);
      assert(!wtypes.empty());
      const auto rtexture_id = reader->claw<MPtr(&Claw::get_sfid)>("texture");
      const auto rtypes = reader->claw<MPtr(&Claw::get_follower_can_do)>(rtexture_id);
      assert(!rtypes.empty());
      // test by ids
      assert(reader->claw<MPtr(&Claw::can_do_swid)>(rtexture_id, writer->get_id(), wtexture_id));
      // test by shmdata types
      bool writer_is_compatible = false;
      for (const auto& it : wtypes) {
        if (reader->claw<MPtr(&Claw::swid_can_do_shmtype)>(rtexture_id, it)) {
          writer_is_compatible = true;
        }
      }
      assert(writer_is_compatible);
      // test if compatible sfids are found with writer shmdata type
      for (const auto& it : wtypes) {
        auto sfids = reader->claw<MPtr(&Claw::get_compatible_sfids)>(it);
        assert(sfids.size() == 2);
        assert(sfids[0] == rtexture_id);
      }
      // test if compatible swids are found with follower shmdata type
      for (const auto& it : rtypes) {
        auto swids = writer->claw<MPtr(&Claw::get_compatible_swids)>(it);
        assert(swids.size() == 1);
        assert(swids[0] == wtexture_id);
      }
    }

    // check connection of the writer "texture" with the reader "texture"
    {
      // get information from the claw + check of consitency among class and Quiddity Tree
      auto wtexture_id = writer->claw<MPtr(&Claw::get_swid)>("texture");
      assert(Ids::kInvalid != wtexture_id);
      assert(wtexture_id == swids["texture"]);
      auto rtexture_id = reader->claw<MPtr(&Claw::get_sfid)>("texture");
      assert(Ids::kInvalid != rtexture_id);
      assert(rtexture_id == sfids["texture"]);

      // connection of both texture shmdata
      auto res_id = reader->claw<MPtr(&Claw::connect)>(rtexture_id, writer->get_id(), wtexture_id);
      // since follower texture is not meta, we expect to obtain the same id
      assert(res_id == rtexture_id);
      // and then disconnect
      assert(reader->claw<MPtr(&Claw::disconnect)>(res_id));
    }

    {
      // check connection to quiddity without mention of a particular shmdata writer
      auto rtexture_id = reader->claw<MPtr(&Claw::get_sfid)>("texture");
      assert(Ids::kInvalid != rtexture_id);
      auto res_id = reader->claw<MPtr(&Claw::connect_quid)>(rtexture_id, writer->get_id());
      // since follower texture is not meta, we expect to obtain the same id
      assert(res_id == rtexture_id);
      // and then disconnect
      assert(reader->claw<MPtr(&Claw::disconnect)>(res_id));
    }

  }  // end of scope is releasing the switcher
  return 0;
}
