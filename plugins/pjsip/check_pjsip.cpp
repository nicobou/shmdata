/*
 * This file is part of switcher-pjsip.
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

#include <unistd.h>             // usleep
#include <assert.h>

#include <vector>
#include <string>
#include <iostream>

#include "switcher/quiddity-manager.hpp"
#include "switcher/quiddity-basic-test.hpp"

#ifdef HAVE_CONFIG_H
#include "../../config.h"
#endif

int main() {
  bool success = true;
  {
    switcher::QuiddityManager::ptr manager =
        switcher::QuiddityManager::make_manager("siptest");

#ifdef HAVE_CONFIG_H
    gchar *usr_plugin_dir = g_strdup_printf("./%s", LT_OBJDIR);
    manager->scan_directory_for_plugins(usr_plugin_dir);
    g_free(usr_plugin_dir);
#else
    return 1;
#endif
   
    // testing uncompressed data transmission
    //g_print ("%s\n",  manager->create("audiotestsrc", "a").c_str());
    assert(0 == manager->create("audiotestsrc", "a").compare ("a"));
    assert(manager->set_property("a", "started", "true"));

    assert(0 == manager->create("videotestsrc", "v").compare ("v"));
    assert(manager->set_property("v", "started", "true"));

    // SIP
    assert(0 == manager->create("sip", "test").compare("test"));
    assert(manager->set_property("test", "port", "5070"));
    
    assert(manager->invoke_va("test",
                              "register",
                              nullptr,
                              "1004",  // user
                              "10.10.30.179",  // domain
                              "1234",  // password
                              nullptr));

    assert(manager->invoke_va("test",
                              "attach_shmdata_to_contact",
                              nullptr,
                              "/tmp/switcher_siptest_a_audio",
                              "blalbal",
                              "true",
                              nullptr));
    assert(manager->invoke_va("test",
                              "attach_shmdata_to_contact",
                              nullptr,
                              "/tmp/switcher_siptest_v_video",
                              "blalbal",
                              "true",
                              nullptr));
    usleep(200000);
    
    assert(manager->invoke_va("test",
                              "call",
                              nullptr,
                              "sip:1002@10.10.30.179",
                              nullptr));

        g_print ("____ %d\n", __LINE__);

    usleep(2000000);
    assert(manager->set_property("test", "status", "Away"));
    usleep(2000000);
    assert(manager->set_property("test", "status-note", "coucou"));
    usleep(2000000);
    assert(manager->set_property("test", "status", "BRB"));
    usleep(2000000);
    assert(manager->invoke_va("test",
                              "hang-up",
                              nullptr,
                              "sip:1002@10.10.30.223",
                              nullptr));
    assert(manager->remove("test"));
  }  // end of scope is releasing the manager

  if (success)
    return 0;
  else
    return 1;
}
