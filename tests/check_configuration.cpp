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

#include "switcher/quiddity/basic-test.hpp"
#include "switcher/switcher.hpp"

using namespace switcher;
namespace fs = std::filesystem;

int main() {
  // 1. check config when making the switcher
  Switcher::ptr mgr = Switcher::make_switcher("test-config", true);

  // check a configuration has been applied
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".logs.filepath").not_null());
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".logs.log_level").not_null());
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".shm.prefix").not_null());

  // 2. load a configuration from a given file. This one contains definition for some bundles
  assert(mgr->conf<MPtr(&Configuration::from_file)>("./check_configuration.json"));

  // check mandatory values in the configuration are available
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".logs.filepath").not_null());
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".logs.log_level").not_null());
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".shm.prefix").not_null());

  // check the bundles from this configuration are available
  assert(mgr->conf<MPtr(&Configuration::list_extra_configs)>().size() == 2);
  assert(quiddity::test::create(mgr, "source-bundle"));

  // 3. load a configuration from an InfoTree
  const auto config_tree = mgr->conf<MPtr(&Configuration::get)>();
  // reapply the same configuration
  assert(mgr->conf<MPtr(&Configuration::from_tree)>(config_tree.get()));

  // check a configuration has been applied
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".logs.filepath").not_null());
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".logs.log_level").not_null());
  assert(mgr->conf<MPtr(&Configuration::get_value)>(".shm.prefix").not_null());

  // check bundles has been reloaded from the config_tree
  assert(mgr->conf<MPtr(&Configuration::list_extra_configs)>().size() == 2);
  assert(quiddity::test::create(mgr, "source-bundle"));

  return 0;
}
