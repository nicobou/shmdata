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

#include "./periodic-task.hpp"

namespace switcher {

PeriodicTask::PeriodicTask(task_t task, std::chrono::milliseconds period):
    task_(task),
    period_(period),
    fut_(task ?
         std::async(std::launch::async, [this](){this->do_work();})
         : std::future<void>()) {
}

PeriodicTask::~PeriodicTask(){
  {
    canceled_.store(true);
    std::unique_lock<std::mutex> lock(cv_m_);
    cv_.notify_one();
  }
  if (fut_.valid()) 
    fut_.get();
}

void PeriodicTask::do_work(){
  while (!canceled_.load()) {
    std::unique_lock<std::mutex> lock(cv_m_);
    if(!cv_.wait_for(lock,
                     period_,
                     [this](){return canceled_.load();})){
      task_();
    }
  }
}

}  // namespace switcher
