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

int main() {
  {
    using namespace switcher;
    using namespace quiddity;
    using namespace claw;
    using namespace property;

    Switcher::ptr sw = Switcher::make_switcher("test_manager");
    assert(quiddity::test::full(sw, "dyn-reader-quid"));

    auto reader = sw->quids<MPtr(&Container::create)>("dyn-reader-quid", "reader", nullptr).get();
    assert(reader);

    auto writer = sw->quids<MPtr(&Container::create)>("connection-quid", "writer", nullptr).get();
    assert(writer);

    // check connection of the writer "texture" with the reader "video%"
    {
      // get information from the claw + check of consitency among class and Quiddity Tree
      auto wtexture_id = writer->claw<MPtr(&Claw::get_swid)>("texture");
      assert(Ids::kInvalid != wtexture_id);
      auto rtexture_id = reader->claw<MPtr(&Claw::get_sfid)>("video%");
      assert(Ids::kInvalid != rtexture_id);

      // check only one follower is specified
      assert(1 == reader->claw<MPtr(&Claw::get_follower_labels)>().size());
      // connection of both texture shmdata
      auto res_id = reader->claw<MPtr(&Claw::connect)>(rtexture_id, writer->get_id(), wtexture_id);
      assert(0 != res_id);
      // since follower texture IS meta, we expect to obtain a different id
      assert(res_id != rtexture_id);
      // check a follower has been created
      assert(2 == reader->claw<MPtr(&Claw::get_follower_labels)>().size());
      // and then disconnect
      assert(reader->claw<MPtr(&Claw::disconnect)>(res_id));
      // check the created follower has been removed
      assert(1 == reader->claw<MPtr(&Claw::get_follower_labels)>().size());
    }

  }  // end of scope is releasing the switcher
  return 0;
}
