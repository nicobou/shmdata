/*
 * This file is part of switcher-video-snapshot.
 *
 * switcher-executor is free software; you can redistribute it and/or
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
#include <condition_variable>
#include <mutex>
#include "switcher/quiddity/basic-test.hpp"
#include "switcher/switcher.hpp"

bool success = false;
std::atomic<bool> do_continue{true};
std::condition_variable cond_var{};
std::mutex mut{};

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

int main() {
  using namespace switcher;
  using namespace quiddity;
  using namespace property;
  Switcher::ptr switcher = Switcher::make_switcher("vsnap");
  switcher->factory<MPtr(&Factory::scan_dir)>("./");

  // performing default tests
  assert(test::full(switcher, "videosnapshot"));

  // creating a video snapshot quiddity
  auto snap = switcher->quids<MPtr(&Container::create)>("videosnapshot", "snap", nullptr).get();

  // configure snap
  snap->prop<MPtr(&PBag::set_str_str)>("imgdir", "/tmp/");
  snap->prop<MPtr(&PBag::set_str_str)>("imgname", "check_video_snapshot");
  snap->prop<MPtr(&PBag::set_str_str)>("num_files", "true");
  snap->prop<MPtr(&PBag::set_str_str)>("quality", "90");

  // ensure we cannot shot without being connected to a video stream
  assert(!snap->prop<MPtr(&PBag::set_str_str)>("shot", "true"));

  // create a video source
  auto vtestsrc = switcher->quids<MPtr(&Container::create)>("videotestsrc", "vid", nullptr).get();
  vtestsrc->prop<MPtr(&property::PBag::set_str_str)>("started", "true");

  // connect VideoSnapshot to the videotestsrc
  assert(snap->meth<MPtr(&method::MBag::invoke_str)>(
      snap->meth<MPtr(&method::MBag::get_id)>("connect"),
      serialize::esc_for_tuple(vtestsrc->make_shmpath("video"))));

  // subscribe to last_image in order to notify success stop after first successful shot
  auto last_image_id = snap->prop<MPtr(&PBag::get_id)>("last_image");
  snap->prop<MPtr(&PBag::subscribe)>(last_image_id, [&]() {
    std::unique_lock<std::mutex> lock(mut);
    success = true;
    do_continue.store(false);
    cond_var.notify_one();
  });

  // trigger the shot
  snap->prop<MPtr(&PBag::set_str_str)>("shot", "true");

  wait_until_success();

  return success ? 0 : 1;
}
