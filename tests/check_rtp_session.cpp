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

#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

#include <gst/gst.h>
#include <cassert>
#include <vector>
#include <string>
#include "switcher/quiddity-manager.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"

static bool audio_success;
static bool video_success;
static bool do_continue;


using namespace switcher;

void on_tree_grafted(const std::string &/*subscriber_name */ ,
                     const std::string &/*quiddity_name */ ,
                     const std::string &signal_name,
                     const std::vector<std::string> &params,
                     void *user_data) {
  auto manager = static_cast<QuiddityManager *>(user_data);
  // std::printf("%s: %s \n", signal_name.c_str(), params[0].c_str());
  GstShmdataSubscriber::num_bytes_t byte_rate =
      //std::string byte_rate =
      manager->use_tree<Any, const std::string &>(
          std::string("uri"), &data::Tree::get_data, params[0] + ".byte_rate");
  if(0 != byte_rate && std::string::npos != params[0].find("audio")){
    audio_success = true;
    do_continue = false;
  }
  std::printf("%s: %s %s\n",
              signal_name.c_str(), params[0].c_str(), std::to_string(byte_rate).c_str());
}

int
main() {
  audio_success = false;
  // video_success = false;
  video_success = true;
  do_continue = true;
  {
    QuiddityManager::ptr manager = QuiddityManager::make_manager("rtptest");
#ifdef HAVE_CONFIG_H
    gchar *usr_plugin_dir = g_strdup_printf("../plugins/gsoap/%s", LT_OBJDIR);
    manager->scan_directory_for_plugins(usr_plugin_dir);
    g_free(usr_plugin_dir);
#else
    return 1;
#endif
    manager->create("SOAPcontrolServer", "soapserver");
    manager->invoke_va("soapserver",
                       "set_port",
                       nullptr,
                       "38084",
                       nullptr);
    // audio
    assert("a" == manager->create("audiotestsrc", "a"));
    manager->set_property("a", "started", "true");
    assert("a2" == manager->create("audiotestsrc", "a2"));
    manager->set_property("a2", "started", "true");
    // // video
    // manager->create("videotestsrc", "v");
    // manager->set_property("v", "started", "true");
    // rtp
    assert("rtp" == manager->create("rtpsession", "rtp"));
    manager->invoke_va("rtp",
                       "add_data_stream",
                       nullptr,
                       "/tmp/switcher_rtptest_a_audio",
                       nullptr);
    manager->invoke_va("rtp",
                       "add_data_stream",
                       nullptr,
                       "/tmp/switcher_rtptest_a2_audio",
                       nullptr);
    // manager->invoke_va("rtp",
    //                    "add_data_stream",
    //                    nullptr,
    //                    "/tmp/switcher_rtptest_v_encoded-video",
    //                    nullptr);
    manager->invoke_va("rtp",
                       "add_destination",
                       nullptr,
                       "local",
                       "127.0.0.1",
                       nullptr);
    manager->invoke_va("rtp",
                       "add_udp_stream_to_dest",
                       nullptr,
                       "/tmp/switcher_rtptest_a_audio",
                       "local",
                       "9066",
                       nullptr);
    manager->invoke_va("rtp",
                       "add_udp_stream_to_dest",
                       nullptr,
                       "/tmp/switcher_rtptest_a2_audio",
                       "local",
                       "9068",
                       nullptr);
    // manager->invoke_va("rtp",
    //                    "add_udp_stream_to_dest",
    //                    nullptr,
    //                    "/tmp/switcher_rtptest_v_encoded-video",
    //                    "local",
    //                    "9076",
    //                    nullptr);
    // receiving
    manager->create("httpsdpdec", "uri");
    manager->invoke_va("uri",
                       "to_shmdata",
                       nullptr,
                       "http://127.0.0.1:38084/sdp?rtpsession=rtp&destination=local",
                       nullptr);
    // checking http-sdp-dec is updating positive byte_rate in tree:
    assert(manager->make_signal_subscriber("signal_subscriber",
                                           on_tree_grafted,
                                           manager.get()));
    assert(manager->subscribe_signal("signal_subscriber",
                                     "uri",
                                     "on-tree-grafted"));
    // wait 3 seconds
    uint count = 3;
    while (do_continue) {
      if (count == 0)
        do_continue = false;
      else {
        --count;
        usleep(1000000);
      }
    }
  }

  gst_deinit();
  if (audio_success && video_success)
    return 0;
  else
    return 1;
}
