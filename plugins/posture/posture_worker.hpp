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

#ifndef __SWITCHER_POSTURE_POOL_HPP__
#define __SWITCHER_POSTURE_POOL_HPP__

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

/*************/
namespace switcher
{
  class Worker
  {
    public:
      Worker();
      ~Worker();
  
      bool is_ready() {return ready_;}
      void set_task(std::function<void()> func);
  
    private:
      std::atomic_bool ready_ {false};
      bool stop_ {false};
      std::shared_ptr<std::function<void()>> task_ {nullptr};
      std::thread thread_ {};
      std::mutex mutex_ {};
      std::condition_variable _condition {};
  
      void thread_loop();
  };
} // end of namespace

#endif
