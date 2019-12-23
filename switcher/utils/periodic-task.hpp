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

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>

namespace switcher {

template <typename T = std::chrono::milliseconds>
class PeriodicTask {
 public:
  using task_t = std::function<void()>;

  PeriodicTask() = delete;
  PeriodicTask(task_t task, T period)
      : task_(task),
        period_(period),
        fut_(task ? std::async(std::launch::async, [this]() { this->do_work(); })
                  : std::future<void>()) {}
  ~PeriodicTask() {
    {
      canceled_.store(true);
      std::unique_lock<std::mutex> lock(cv_m_);
      cv_.notify_one();
    }
  }

 private:
  task_t task_;
  T period_;
  std::condition_variable cv_{};
  std::mutex cv_m_{};
  std::atomic<bool> canceled_{false};
  std::future<void> fut_{};

  void do_work() {
    auto exec_duration = std::chrono::system_clock::duration(0);
    while (!canceled_.load()) {
      std::unique_lock<std::mutex> lock(cv_m_);
      auto wait_duration = period_ - exec_duration;
      // If this happens it means the task is too long for the desired period.
      if (wait_duration.count() < 0) wait_duration = std::chrono::system_clock::duration(0);
      if (!cv_.wait_for(lock, wait_duration, [this]() { return canceled_.load(); })) {
        auto start_clock = std::chrono::system_clock::now();
        task_();
        exec_duration = std::chrono::system_clock::now() - start_clock;
      }
    }
  }
};

}  // namespace switcher
#endif
