/*
 * This file is part of posture.
 *
 * posture is free software; you can redistribute it and/or
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

#include "posture_worker.hpp"

#include <iostream>

using namespace std;
namespace switcher {

/*************/
Worker::Worker() {
  thread_ = thread([&]() { thread_loop(); });
}

/*************/
Worker::~Worker() {
  stop_ = true;
  thread_.join();
}

/*************/
void Worker::set_task(function<void()> func) {
  lock_guard<mutex> lock(mutex_);
  ready_ = false;
  task_ = make_shared<function<void()>>(func);
  _condition.notify_one();
}

/*************/
void Worker::thread_loop() {
  while (true) {
    unique_lock<mutex> lock(mutex_);
    ready_ = true;
    _condition.wait(lock);

    if (task_ == nullptr) continue;

    if (stop_) return;

    (*task_)();
    task_.reset();
  }
}

}  // end of namespace
