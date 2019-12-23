/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef NDEBUG  // get assert in release mode

#include <gst/gst.h>
#include <cassert>
#include "switcher/gst/unique-gst-element.hpp"

int main() {
  using namespace switcher;

  gst_init(nullptr, nullptr);
  {
    UGstElem elem("tee");
    if (!elem) return 1;
  }
  {
    UGstElem elem("non-existing-element");
    if (elem) return 1;
  }
  {
    UGstElem elem1("tee");
    UGstElem elem2("tee");
    elem2 = std::move(elem1);
    if (elem1 || !elem2) return 1;
  }
  {
    UGstElem elem("tee");
    bool success = false;
    if (!elem.register_notify_on_property_change("name", [&success]() { success = !success; }))
      return 1;
    g_object_set(G_OBJECT(elem.get_raw()), "name", "test_name", nullptr);
    assert(success);
    if (!elem.unregister_notify_on_property_change("name")) return 1;
    g_object_set(G_OBJECT(elem.get_raw()), "name", "test_name_changed", nullptr);
    assert(success);
  }
  gst_deinit();
  return 0;
}
