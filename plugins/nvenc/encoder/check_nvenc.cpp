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
#include "switcher/quiddity/basic-test.hpp"
#include "switcher/shmdata/gst-subscriber.hpp"

#include "cuda/cuda-context.hpp"

static bool success = false;
static std::atomic<bool> do_continue{true};
static std::condition_variable cond_var{};
static std::mutex mut{};

using namespace switcher;
using namespace quiddity;

void wait_until_success() {
  // wait 3 seconds
  unsigned int count = 3;
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
    auto devices = quiddities::CudaContext::get_devices();
    if (devices.empty()) {
      return 0; // no NVENC-enabled GPU as been detected
    }

    Switcher::ptr manager = Switcher::make_switcher("test_manager");

    manager->factory<MPtr(&quiddity::Factory::scan_dir)>("./");

    // testing if two nvenc can be created simultaneously
    std::vector<std::string> nvencs;
    for (int i = 0; i < 2; ++i) {
      auto created =
          manager->quids<MPtr(&quiddity::Container::create)>("nvenc", std::string(), nullptr);
      if (!created) {
        std::cerr << "nvenc creating failed, i: " << i << '\n';
        return 1;
      }
      nvencs.push_back(created.msg());
    }
    for (auto& it : nvencs)
      manager->quids<MPtr(&quiddity::Container::remove)>(
          manager->quids<MPtr(&quiddity::Container::get_id)>(it));
    nvencs.clear();

    // testing if nvenc can be used
    {
      auto created =
          manager->quids<MPtr(&quiddity::Container::create)>("nvenc", std::string(), nullptr);
      if (!created) {
        std::cerr << "nvenc encoding could not be created" << '\n';
        return 1;
      }
      manager->quids<MPtr(&quiddity::Container::remove)>(
          manager->quids<MPtr(&quiddity::Container::get_id)>(created.msg()));
    }

    // standard test
    assert(quiddity::test::full(manager, "nvenc"));
    manager->quids<MPtr(&quiddity::Container::remove)>(
        manager->quids<MPtr(&quiddity::Container::get_id)>("nvenc"));

    // testing nvenc is encoding
    auto created =
        manager->quids<MPtr(&quiddity::Container::create)>("videotestsrc", std::string(), nullptr);
    assert(created);
    auto vid = created.get();
    vid->prop<MPtr(&property::PBag::set_str_str)>("codec", "0");
    vid->prop<MPtr(&property::PBag::set_str_str)>("started", "true");
    auto vid_shmpath = vid->make_shmpath("video");
    assert(!vid_shmpath.empty());

    auto nvenc_created =
        manager->quids<MPtr(&quiddity::Container::create)>("nvenc", std::string(), nullptr);
    assert(nvenc_created);
    auto nvenc = nvenc_created.get();
    assert(nvenc);
    nvenc->meth<MPtr(&method::MBag::invoke_str)>(
        nvenc->meth<MPtr(&method::MBag::get_id)>("connect"), serialize::esc_for_tuple(vid_shmpath));

    // tracking nvenc shmdata writer byterate for evaluating success
    auto registration_id = nvenc->sig<MPtr(&signal::SBag::subscribe_by_name)>(
        "on-tree-grafted", [&](const InfoTree::ptr& tree) {
          size_t byte_rate = nvenc->tree<MPtr(&InfoTree::branch_get_value)>(
              tree->get_value().as<std::string>() + ".byte_rate");
          if (0 != byte_rate) {
            notify_success();
          }
        });
    assert(0 != registration_id);

    wait_until_success();
    assert(
        nvenc->sig<MPtr(&signal::SBag::unsubscribe_by_name)>("on-tree-grafted", registration_id));
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
    manager->factory<MPtr(&quiddity::Factory::scan_dir)>("./");
    manager->conf<MPtr(&Configuration::from_file)>("./check_decode.json");

    auto nvdec_created =
        manager->quids<MPtr(&quiddity::Container::create)>("nvencdecoder", "nvencdecoder", nullptr);
    assert(nvdec_created);
    auto nvdec = nvdec_created.get();
    assert(nvdec);
    nvdec->prop<MPtr(&property::PBag::set_str_str)>("started", "true");
    nvdec->prop<MPtr(&property::PBag::subscribe)>(
        nvdec->prop<MPtr(&property::PBag::get_id)>("dummy/frame-received"), [&]() {
          success = nvdec->prop<MPtr(&property::PBag::get<bool>)>(
              nvdec->prop<MPtr(&property::PBag::get_id)>("dummy/frame-received"));
          notify_success();
        });

    wait_until_success();
  }

  gst_deinit();
  if (success)
    return 0;
  return 1;
}
