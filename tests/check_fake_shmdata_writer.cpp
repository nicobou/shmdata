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

//#include <gst/gst.h>
#include <string>
#include <cassert>
#include "switcher/quiddity-manager.hpp"
#include "switcher/information-tree.hpp"

static bool success = false; 
void
property_cb(std::string /*subscriber_name*/,
            std::string /*quiddity_name*/,
            std::string /*property_name*/,
            std::string /*value*/,
            void */*user_data*/) {
  success = true;  // got a caps, success
}

int
main() {
  {
    // creating an audiotestsrc, getting the shmdata path and
    // giving it to fakeshmdatasrc 
    
    switcher::QuiddityManager::ptr manager =
        switcher::QuiddityManager::make_manager("check-fakesink");
    //preparing fakeshmsrc and an audio
    assert("fakeshmsrc" == manager->create("fakeshmsrc", "fakeshmsrc"));
    assert("audio" == manager->create("audiotestsrc", "audio"));
    assert(manager->set_property("audio", "started", "true"));

    // FIXME synchronize with audio registration of audio shmdata writer
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // getting audio shmdata name for fakeshmsrc
    std::list<std::string> audio_shmdata = manager->
        invoke_info_tree<std::list<std::string>>(
            "audio",
            [&](switcher::data::Tree::ptrc tree){
              return tree->get_child_keys<std::list>(".shmdata.writer.");
            });
    assert(!audio_shmdata.empty());
    //assert(manager->set_property("audio", "started", "false"));

    // getting fakeshmsrc with the audio shmdata
    manager->set_property("fakeshmsrc", "shmdata-path", (*audio_shmdata.begin()).c_str());
    manager->set_property("fakeshmsrc", "started", "true");

   // FIXME synchronize with audio registration of audio shmdata writer
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
     
    // getting audio shmdata name for fakeshmsrc
    std::list<std::string> fakeshm_shm = manager->
        invoke_info_tree<std::list<std::string>>(
            "fakeshmsrc",
            [&](switcher::data::Tree::ptrc tree){
              return tree->get_child_keys<std::list>(".shmdata.writer.");
            });
    // FIXME assert(!fakeshm_shm.empty());
    // checking fakeshmsrc is exposing the same shmdata path
    //FIXME assert((*audio_shmdata.begin()) == (*fakeshm_shm.begin()));

  }  // releasing manager

  gst_deinit();
  return 0;
}
