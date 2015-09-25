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

using namespace std;
namespace switcher
{

  /*************/
  Worker::Worker()
  {
    thread_ = thread([&] () {
      thread_loop();
    });
  }
  
  /*************/
  Worker::~Worker()
  {
    stop_ = true;
    thread_.join();
  }
  
  /*************/
  void Worker::set_task(function<void()> func)
  {
    lock_guard<mutex> lock(mutex_);
    task_ = make_shared<function<void()>>(func);
  }
  
  /*************/
  void Worker::do_task()
  {
    lock_guard<mutex> lock(mutex_);
    if (task_ != nullptr)
      do_task_ = true;
  }
  
  /*************/
  void Worker::thread_loop()
  {
    while (true)
    {
      ready_ = true;

      while (!stop_ && (task_ == nullptr || do_task_ == false))
        this_thread::sleep_for(chrono::milliseconds(1));
  
      if (stop_)
        return;
  
      lock_guard<mutex> lock(mutex_);
      ready_ = false;
      do_task_ = false;
  
      (*task_)();
      task_.reset();
    }
  }

} // end of namespace
