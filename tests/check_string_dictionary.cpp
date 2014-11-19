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

#include <glib.h>  // g_remove
#include <glib/gstdio.h>  // g_remove
#include <cassert>
#include "switcher/quiddity-manager.hpp"

int
main() {

  switcher::QuiddityManager::ptr manager =
      switcher::QuiddityManager::make_manager("dicotest");
  
  // creating a dico and saving it 
    manager->create("dico", "dic");
    manager->invoke_va("dic",
                       "update",
                       nullptr,
                       "me@example.net",
                       "1234",
                       nullptr);

    g_print("%s\n",manager->get_info("dic", ".").c_str());

    {  // reading with original name
      std::string *res = nullptr;
      manager->invoke_va("dic",
                         "read",
                         &res,
                         "me@example.net",
                         nullptr);
      assert(nullptr != res && (*res) == "1234");
      delete res;
    }

    {  // reading with dot-escaped name
      std::string *res = nullptr;
      manager->invoke_va("dic",
                         "read",
                         &res,
                         "me@example__DOT__net",
                         nullptr);
    assert(nullptr != res && (*res) == "1234");
    delete res;
    }

    
    manager->invoke_va("dic",
                       "save",
                       nullptr,
                       "./dico.save",
                       nullptr);
    manager->invoke_va("dic",
                       "remove",
                       nullptr,
                       "me@example.net",
                       nullptr);

    // creating an other dico and loading from the saved file 
    manager->create("dico", "dic2");
    manager->invoke_va("dic2",
                       "load",
                       nullptr,
                       "./dico.save",
                       nullptr);

    {  // reading with original name
      std::string *res = nullptr;
      manager->invoke_va("dic2",
                         "read",
                         &res,
                         "me@example.net",
                         nullptr);
      assert(nullptr != res && (*res) == "1234");
      delete res;
    }
    
    {  // reading with dot-escaped name
      std::string *res = nullptr;
      manager->invoke_va("dic2",
                         "read",
                         &res,
                         "me@example__DOT__net",
                         nullptr);
      assert(nullptr != res && (*res) == "1234");
      delete res;
    }
    
    assert(0 == g_remove("./dico.save"));
    return 0;
}
