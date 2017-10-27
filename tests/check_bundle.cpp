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

#include <cassert>
#include "switcher/switcher.hpp"

using namespace switcher;

int main() {
  {
    Switcher::ptr manager = Switcher::make_switcher("bundle");
    assert(manager->load_configuration_file("./check_bundle.config"));
    assert(manager->create("source-bundle", "source-bundle") == "source-bundle");
    assert(manager->create("sink-bundle", "sink-bundle") == "sink-bundle");
    assert(manager->create("filter-bundle", "filter-bundle") == "filter-bundle");
  }
  gst_deinit();
  return 0;
}
