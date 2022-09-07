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
#include <vector>

#include "switcher/quiddity/basic-test.hpp"
#include "switcher/utils/string-utils.hpp"

int main() {
  {
    using namespace switcher;
    using namespace quiddity;
    using namespace claw;
    using namespace property;

    Switcher::ptr sw = Switcher::make_switcher("test_manager");
    assert(quiddity::test::full(sw, "dyn-writer-quid"));

    auto dynw = sw->quids<MPtr(&Container::create)>("dyn-writer-quid", "writer", nullptr).get();
    assert(dynw);
    // register to new connection spec in the tree
    int num_added_received = 0;
    assert(0 !=
           dynw->sig<MPtr(&signal::SBag::subscribe_by_name)>(
               "on-connection-spec-added", [&](const InfoTree::ptr& tree) {
                 assert(stringutils::starts_with(tree->read_data().as<std::string>(), "writer."));
                 ++num_added_received;
               }));
    int num_removed_received = 0;
    assert(0 !=
           dynw->sig<MPtr(&signal::SBag::subscribe_by_name)>(
               "on-connection-spec-removed", [&](const InfoTree::ptr& tree) {
                 assert(stringutils::starts_with(tree->read_data().as<std::string>(), "writer."));
                 ++num_removed_received;
               }));

    auto reader = sw->quids<MPtr(&Container::create)>("connection-quid", "reader", nullptr).get();
    assert(reader);

    // 1 - dynw has no writer enabled: check nothing connects
    {
      // dynw is expected tp get only one writer, i.e. video%
      assert(1 == dynw->claw<MPtr(&Claw::get_writer_labels)>().size());

      // check the reader quiddity is not able to connect to the meta-writer
      auto res_id =
          reader->claw<MPtr(&Claw::connect)>(reader->claw<MPtr(&Claw::get_sfid)>("texture"),
                                             dynw->get_id(),
                                             dynw->claw<MPtr(&Claw::get_swid)>("video%"));
      assert(Ids::kInvalid == res_id);
    }  // end of 1 - dynw has no writer enabled: check nothing connects

    // 2 - enable the dynamic shmdata writer and check connection success
    {
      assert(dynw->prop<MPtr(&PBag::set_str_str)>("video", "true"));
      // check we received two notifications
      assert(2 == num_added_received);

      // inspect dynw
      std::vector<swid_t> swids;
      {
        auto conspec_tree = dynw->conspec<MPtr(&InfoTree::get_copy)>();
        auto meta_shmdata_swid = dynw->claw<MPtr(&Claw::get_swid)>("video%");
        conspec_tree->cfor_each_in_array("writer", [&](const InfoTree* tree) {
          const auto label = tree->branch_get_value("label").as<std::string>();
          // check descriptions
          if (label == "videoA")
            assert(tree->branch_get_value("description").as<std::string>() ==
                   "A is a video Shmdata");
          if (label == "videoB")
            assert(tree->branch_get_value("description").as<std::string>() ==
                   "B is a video Shmdata");
          // check from_swid is correct
          const auto from_swid = tree->branch_get_value("from_swid").as<swid_t>();
          if (label == "videoA" || label == "videoB") {
            swids.emplace_back(tree->branch_get_value("swid"));
            assert(meta_shmdata_swid == from_swid);
          } else {
            assert(Ids::kInvalid == from_swid);
          }
        });
        assert(2 == swids.size());  // only video A + video B
      }

      // check the reader quiddity is able to connect to video A and video B
      auto rtexture_id = reader->claw<MPtr(&Claw::get_sfid)>("texture");
      for (const auto& swid : swids) {
        auto res_id = reader->claw<MPtr(&Claw::connect)>(rtexture_id, dynw->get_id(), swid);
        assert(Ids::kInvalid != res_id);
        assert(reader->claw<MPtr(&Claw::disconnect)>(res_id));
      }
    }  // end of 2 - enable the dynamic shmdata writer and check connection success

    // 3 stop these dynamic writers
    assert(dynw->prop<MPtr(&PBag::set_str_str)>("video", "false"));
    // check we received two notifications
    assert(2 == num_removed_received);

  }  // end of scope is releasing the switcher
  return 0;
}
