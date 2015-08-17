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

#ifndef __SWITCHER_PERIODIC_TASK_H__
#define __SWITCHER_PERIODIC_TASK_H__

#include <future>
#include <atomic>
#include <chrono>

namespace switcher {

class PeriodicTask {
  using task_t = std::function<void()>;   

 public:
  PeriodicTask() = delete;
  PeriodicTask(task_t task, std::chrono::milliseconds period);
  ~PeriodicTask();

 private:
  task_t task_;
  std::chrono::milliseconds period_;
  std::condition_variable cv_{};
  std::mutex cv_m_{};
  std::atomic<bool> canceled_{false};
  std::future<void> fut_{};
  void do_work();
};

}  // namespace switcher
#endif
