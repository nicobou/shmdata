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
  // get default configuration file path
  auto global_path = Configuration::get_default_global_path();

  // backup the file if any, and replace it with "./check_configuration.json"
  auto global_path_backup = global_path;
  global_path_backup += ".to_restore";
  bool do_restore_initial_config_file = false;
  if (fs::exists(global_path)) {
    fs::copy_file(global_path, global_path_backup);
    do_restore_initial_config_file = true;
    fs::remove(global_path);
  }
  fs::create_directories(global_path.parent_path());
  fs::copy_file("./check_configuration.json", global_path);

  // making the switcher
  Switcher::ptr mgr = Switcher::make_switcher("configuration", true);

  // check the configuration has been applied
  assert(mgr->conf<MPtr(&Configuration::list_extra_configs)>().size() == 2);
  assert(quiddity::test::create(mgr, "source-bundle"));

  // cleanup and restore previous config file if any
  fs::remove(global_path);
  if (do_restore_initial_config_file) {
    fs::copy_file(global_path_backup, global_path);
    fs::remove(global_path_backup);
  }

  return 0;
}
