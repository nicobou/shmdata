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
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/switcher.hpp"

static bool audio_success = false;
static bool audio_added = false;
static bool audio_to_rtp_done = false;
static const std::string rtp_port = std::string("9066");
// waiting for success
std::mutex cv_m{};
std::condition_variable cv{};

using namespace switcher;

void on_tree_grafted(const std::string& /*subscriber_name */,
                     const std::string& /* quiddity_name */,
                     const std::string& signal_name,
                     const std::vector<std::string>& params,
                     void* user_data) {
  auto manager = static_cast<Switcher*>(user_data);
  // interested here in byte_rate information from httpsdpdec
  if (std::string::npos == params[0].find("stat")) return;
  // check the received byte rate
  size_t byte_rate = manager->use_tree<MPtr(&InfoTree::branch_get_value)>(std::string("uri"),
                                                                          params[0] + ".byte_rate");
  if (0 != byte_rate) {
    audio_success = true;
    cv.notify_one();
    std::printf(
        "%s: %s %s\n", signal_name.c_str(), params[0].c_str(), std::to_string(byte_rate).c_str());
  }
}

void receive_audio(Switcher* manager) {
  // configure an httpsdpdec
  assert(manager->create("httpsdpdec", "uri") == "uri");
  manager->invoke_va("uri",
                     "to_shmdata",
                     nullptr,
                     "http://127.0.0.1:38084/sdp?rtpsession=rtp&destination=local",
                     nullptr);
  // checking http-sdp-dec is updating positive byte_rate in tree
  assert(manager->make_signal_subscriber("signal_subscriber", on_tree_grafted, manager));
  assert(manager->subscribe_signal("signal_subscriber", "uri", "on-tree-grafted"));
}

void on_audio_info(const std::string& /*subscriber_name */,
                   const std::string& quiddity_name,
                   const std::string& /*signal_name*/,
                   const std::vector<std::string>& params,
                   void* user_data) {
  // ignoring information not related to byte_rate
  if (std::string::npos == params[0].find("stat")) return;
  auto manager = static_cast<Switcher*>(user_data);
  // checking for shmdata writer path
  auto audio_shmpath =
      manager->use_tree<MPtr(&InfoTree::get_child_keys)>(quiddity_name, "shmdata.writer");
  if (audio_shmpath.empty()) return;
  if (!audio_to_rtp_done) {
    size_t byte_rate = manager->use_tree<MPtr(&InfoTree::branch_get_value)>(
        quiddity_name, std::string("shmdata.writer.") + audio_shmpath.front() + ".stat.byte_rate");
    if (!audio_added) {
      manager->invoke_va("rtp", "add_data_stream", nullptr, audio_shmpath.front().c_str(), nullptr);
      audio_added = true;
    }
    if (byte_rate != 0) {
      auto thread = std::thread([=]() {
        manager->invoke_va("rtp",
                           "add_udp_stream_to_dest",
                           nullptr,
                           audio_shmpath.front().c_str(),
                           "local",
                           rtp_port.c_str(),
                           nullptr);
        audio_to_rtp_done = true;
        // ready to:
        receive_audio(manager);
      });
      thread.detach();
    }
  }
}

int main() {
  {
    // this lock is used with a condition variable that notifies success
    std::unique_lock<std::mutex> lock(cv_m);

    Switcher::ptr manager = Switcher::make_switcher("rtptest");

    manager->scan_directory_for_plugins("./");

    // creating a SOAP server that will also distribute SDP file through the 'magic' url
    // magic url is
    // http://<soap_server>:<soap_port>/sdp?rtpsession=<rtp_quiddity_name>&destination=<rtp_destination_name>
    manager->create("SOAPcontrolServer", "soapserver");
    manager->invoke_va("soapserver", "set_port", nullptr, "38084", nullptr);

    // creating rtp quiddity
    assert("rtp" == manager->create("rtpsession", "rtp"));
    manager->invoke_va("rtp", "add_destination", nullptr, "local", "127.0.0.1", nullptr);

    // creating two audio shmdatas
    std::string audio_quid = "a";
    assert(audio_quid == manager->create("audiotestsrc", audio_quid.c_str()));
    manager->use_prop<MPtr(&PContainer::set_str_str)>(audio_quid.c_str(), "started", "true");

    assert(manager->make_signal_subscriber("audio_subscriber", on_audio_info, manager.get()));
    assert(manager->subscribe_signal("audio_subscriber", audio_quid.c_str(), "on-tree-grafted"));

    // stop waiting result after 5 seconds
    cv.wait_for(lock, std::chrono::seconds(8));
  }

  gst_deinit();
  if (audio_success)
    return 0;
  else
    return 1;
}
