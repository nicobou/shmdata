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

#include <gst/gst.h>
#include <string>
#include <cassert>
#include "switcher/quiddity-manager.hpp"

static bool success = false; 
void
property_cb(std::string subscriber_name,
            std::string quiddity_name,
            std::string property_name,
            std::string value,
            void *user_data) {
  // got a caps, success
  success = true;
  g_debug("%s %s %s %s\n",
          subscriber_name.c_str(),
          quiddity_name.c_str(),
          property_name.c_str(),
          value.c_str());
}

int
main() {
  {
    switcher::QuiddityManager::ptr manager =
        switcher::QuiddityManager::make_manager("check-fake-shmdata-writer");
    //preparing fakesink
    assert("vu" == manager->create("fakesink", "vu"));

    //preparing audio source for testing
    assert("audio" == manager->create("audiotestsrc", "audio"));
    assert(manager->set_property("audio", "started", "true"));

    // connecting/disconnecting fakesink with audio source
    assert(manager->invoke_va("vu", "connect", nullptr,
                              "/tmp/switcher_check-fake-shmdata-writer_audio_audio", nullptr));
    assert(manager->invoke_va("vu", "disconnect-all", nullptr, nullptr));
    assert(manager->invoke_va("vu", "connect", nullptr,
                              "/tmp/switcher_check-fake-shmdata-writer_audio_audio", nullptr));

    // stoping audio and removing the fakesink "vu"
    assert(manager->set_property("audio", "started", "false"));
    manager->remove("vu");
    manager->set_property("audio", "started", "true");

    //preparing a new fakesink with subscription to caps
    assert("vu" == manager->create("fakesink", "vu"));
    assert(manager->make_property_subscriber("sub", property_cb, nullptr));
    assert(manager->subscribe_property("sub", "vu", "caps"));
    assert(manager->invoke_va("vu", "connect", nullptr,
                              "/tmp/switcher_check-fake-shmdata-writer_audio_audio", nullptr));

    // waiting for caps being received. After 2 seconds, fail.
    int count = 20;
    while (--count > -1) {
      if (!success)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      else
        count = 0;
    }
    
    assert(manager->remove("vu"));
  }  // releasing manager

  gst_deinit();
  if (success)
    return 0;
  return 1;
}
