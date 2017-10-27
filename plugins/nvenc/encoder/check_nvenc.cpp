/*
 * This file is part of switcher-myplugin.
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

#undef NDEBUG  // get assert in release mode

#include <atomic>
#include <cassert>
#include <chrono>
#include <future>
#include <vector>
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/quiddity-basic-test.hpp"

static bool success = false;
static std::atomic<bool> do_continue{true};
static std::condition_variable cond_var{};
static std::mutex mut{};

using namespace switcher;

void wait_until_success() {
  // wait 3 seconds
  uint count = 3;
  while (do_continue.load()) {
    std::unique_lock<std::mutex> lock(mut);
    if (count == 0) {
      do_continue.store(false);
    } else {
      --count;
      cond_var.wait_for(lock, std::chrono::seconds(1), []() { return !do_continue.load(); });
    }
  }
}

void notify_success() {
  std::unique_lock<std::mutex> lock(mut);
  success = true;
  do_continue.store(false);
  cond_var.notify_one();
}

int main() {
  {
    Switcher::ptr manager = Switcher::make_switcher("test_manager");

    manager->scan_directory_for_plugins("./");

    // testing if two nvenc can be created simultaneously
    std::vector<std::string> nvencs;
    for (int i = 0; i < 2; ++i) {
      auto nvenc_quid = manager->create("nvenc");
      if (nvenc_quid.empty()) {
        std::cerr << "nvenc creating failed, i: " << i << '\n';
        return 1;
      }
      nvencs.push_back(nvenc_quid);
    }
    for (auto& it : nvencs) manager->remove(it);
    nvencs.clear();

    // testing if nvenc can be used
    {
      auto nvenc_quid = manager->create("nvenc");
      if (nvenc_quid.empty()) {
        std::cerr << "nvenc encoding could not be created" << '\n';
        return 1;
      }
      manager->remove(nvenc_quid);
    }

    // standard test
    assert(QuiddityBasicTest::test_full(manager, "nvenc"));
    manager->remove("nvenc");


    // testing nvenc is encoding
    auto video_quid = manager->create("videotestsrc");
    assert(!video_quid.empty());
    manager->use_prop<MPtr(&PContainer::set_str_str)>(video_quid.c_str(), "codec", "0");
    manager->use_prop<MPtr(&PContainer::set_str_str)>(video_quid.c_str(), "started", "true");
    // wait for video to be started
    usleep(100000);
    auto vid_shmdata_list =
        manager->use_tree<MPtr(&InfoTree::get_child_keys)>(video_quid.c_str(), "shmdata.writer");
    auto vid_shmpath = vid_shmdata_list.front();
    assert(!vid_shmpath.empty());

    auto nvenc_quid = manager->create("nvenc");
    assert(!nvenc_quid.empty());
    manager->invoke_va(nvenc_quid.c_str(), "connect", nullptr, vid_shmpath.c_str(), nullptr);

    // tracking nvenc shmdata writer byterate for evaluating success
    auto registration_id = manager->use_sig<MPtr(&switcher::SContainer::subscribe_by_name)>(
        nvenc_quid, "on-tree-grafted", [&](const switcher::InfoTree::ptr& tree) {
          size_t byte_rate = manager->use_tree<MPtr(&InfoTree::branch_get_value)>(
              nvenc_quid, tree->get_value().as<std::string>() + ".byte_rate");
          if (0 != byte_rate) {
            notify_success();
          }
        });
    assert(0 != registration_id);

    wait_until_success();
    assert(manager->use_sig<MPtr(&switcher::SContainer::unsubscribe_by_name)>(
        nvenc_quid, "on-tree-grafted", registration_id));
  }  // end of scope is releasing the manager

  if (!success) {
    std::cerr << " error line " << __LINE__ << std::endl;
    return 1;
  }

  {
    // re-init test
    do_continue.store(true);
    success = false;
    // starting a new test: nvenc data can be decoded
    Switcher::ptr manager = Switcher::make_switcher("test_manager");
    manager->scan_directory_for_plugins("./");
    manager->load_configuration_file("./check_decode.json");

    auto nvencdec = manager->create("nvencdecoder", "nvencdecoder");
    assert(nvencdec == "nvencdecoder");
    manager->use_prop<MPtr(&PContainer::set_str_str)>(nvencdec, "started", "true");
    manager->use_prop<MPtr(&PContainer::subscribe)>(
        nvencdec,
        manager->use_prop<MPtr(&PContainer::get_id)>(nvencdec, "dummy/frame-received"),
        [&]() {
          success = manager->use_prop<MPtr(&PContainer::get<bool>)>(
              nvencdec,
              manager->use_prop<MPtr(&PContainer::get_id)>(nvencdec, "dummy/frame-received"));
          notify_success();
        });

    wait_until_success();
  }

  gst_deinit();
  if (success)
    return 0;
  else
    return 1;
}
