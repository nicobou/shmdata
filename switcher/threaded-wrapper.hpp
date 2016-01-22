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

#ifndef __SWITCHER_THREADED_WRAPPER_H__
#define __SWITCHER_THREADED_WRAPPER_H__

#include <functional>
#include <future>
#include <atomic>
#include <vector>
#include <memory>
#include <iostream>

namespace switcher {
template<typename T>
class ThreadedWrapper{
 public:
  template<typename ...Args>
  ThreadedWrapper(Args... args):
      fut_ (std::async(std::launch::async,
                       [&](){
                         threaded_wrap();
                       })){
    { std::unique_lock<std::mutex> lock(start_m_);
      start_cv_.wait(lock); }
    do_sync_task([&](){
        member_.reset(new T(args...));
      });
  }
  ~ThreadedWrapper(){
    do_sync_task([&](){
        run_async_tasks();
        // destroy member from the thread:
        member_.reset(nullptr);
        // stop the thread
        canceled_.store(true);
      });
    // wait for the thread
    if (fut_.valid()) 
      fut_.get();
  }

  
  void threaded_wrap(){
    // work loop
    while (!canceled_.load()) {
      std::unique_lock<std::mutex> lock(cv_m_);
      if (!started_){
        std::unique_lock<std::mutex> lock(start_m_);
        start_cv_.notify_one();
        started_ = true;
      }
      // sync task
      if(sync_task_){
        sync_task_();
        sync_task_ = nullptr;
        std::unique_lock<std::mutex> lock2(task_done_m_);
        task_done_ = true;
        task_done_cv_.notify_one();
      }
      // async tasks
      run_async_tasks();
      cv_.wait_for(lock, std::chrono::milliseconds(1));
    }
  }
  
  // introspection of methods return type
  template<typename F, F f>
  struct method_traits;
  // const
  template<typename C, typename R, typename ...Args, R(C::*fn_ptr)(Args...) const>
  struct method_traits<R(C::*)(Args...) const, fn_ptr>{using return_t = R;};
  // non const
  template<typename C, typename R, typename ...Args, R(C::*fn_ptr)(Args... )>
  struct method_traits<R(C::*)(Args...), fn_ptr>{ using return_t = R; };

  // returning methods
  template<typename MType, MType fun, typename ...ARGs,
           typename std::enable_if<
             !std::is_same<void, typename method_traits<MType, fun>::return_t>::value
             >::type* = nullptr>
  typename method_traits<MType, fun>::return_t invoke(ARGs ...args){
    typename method_traits<MType, fun>::return_t res;
    auto task = [&](){
      res = (this->member_.get() ->* fun)(std::forward<ARGs>(args)...);
      std::unique_lock<std::mutex> lock(this->task_done_m_);
      this->task_done_cv_.notify_one();
    };
    do_sync_task(task);
    return res;
  }

  template<typename MType, MType fun, typename ...ARGs,
           typename std::enable_if<
             !std::is_same<void, typename method_traits<MType, fun>::return_t>::value
             >::type* = nullptr>
  void invoke_async(
      std::function<void(typename method_traits<MType, fun>::return_t)>  on_result,
      ARGs ...args){
    { std::unique_lock<std::mutex> lock_sync(async_mtx_);
      async_tasks_.emplace_back([=](){
          if (on_result)
            on_result((this->member_.get() ->* fun)(args...));
          else
            (this->member_.get() ->* fun)(args...);
        });
    }
    return;
  }

  // void methods
  template<typename MType, MType fun, typename ...ARGs,
           typename std::enable_if<
             std::is_same<void, typename method_traits<MType, fun>::return_t>::value
             >::type* = nullptr>
  typename method_traits<MType, fun>::return_t invoke(ARGs ...args){
    auto task = [&](){
      (this->member_.get() ->* fun)(std::forward<ARGs>(args)...);
      std::unique_lock<std::mutex> lock(this->task_done_m_);
      this->task_done_cv_.notify_one();
    };
    do_sync_task(task);
    return;
  }

  template<typename MType, MType fun, typename ...ARGs,
           typename std::enable_if<
             std::is_same<void, typename method_traits<MType, fun>::return_t>::value
             >::type* = nullptr>
  void invoke_async(
      std::function<void()>  on_result,
      ARGs ...args){
    { std::unique_lock<std::mutex> lock_sync(async_mtx_);
      async_tasks_.emplace_back([=](){
          if (on_result) {
            (this->member_.get() ->* fun)(args...);
            on_result();
          } else {
            (this->member_.get() ->* fun)(args...);
          }
        });
    }
    return;
  }

  //returning functions
  template<typename R,
           typename std::enable_if<
             !std::is_same<void, R>::value
             >::type* = nullptr>
  R run(std::function<R()> fun){
    R res;
    auto task = [&](){
      res = fun();
      std::unique_lock<std::mutex> lock(this->task_done_m_);
      this->task_done_cv_.notify_one();
    };
    do_sync_task(task);
    return res;
  }

  template<typename R,
           typename std::enable_if<
             !std::is_same<void, R>::value>::type* = nullptr>
  void run_async(std::function<void()> fun, std::function<void(R)> on_result){
    { std::unique_lock<std::mutex> lock_sync(async_mtx_);
      async_tasks_.emplace_back([=](){
          if (on_result) on_result(fun());
          else fun();
        });
    }
    return;
  }

  // void functions
  template<typename R = void,
           typename std::enable_if<std::is_same<void, R>::value>::type* = nullptr>
  R run(std::function<void()> fun){
    auto task = [&](){
      fun();
      std::unique_lock<std::mutex> lock(this->task_done_m_);
      this->task_done_cv_.notify_one();
    };
    do_sync_task(task);
    return;
  }

  template<typename R = void,
           typename std::enable_if<std::is_same<void, R>::value>::type* = nullptr>
  void run_async(std::function<void()> fun,
                 std::function<void()> on_done = nullptr){
    { std::unique_lock<std::mutex> lock_sync(async_mtx_);
      async_tasks_.emplace_back([=](){
          if (on_done) {
            fun();
            on_done();
          } else {
            fun();
          }
        });
    }
    return;
  }
  
 private:
  std::unique_ptr<T> member_;
  // sync ctor
  std::mutex start_m_{};
  std::condition_variable start_cv_{};
  bool started_{false};
  // sync task
  std::function<void()> sync_task_{nullptr};
  std::mutex cv_m_{};
  std::condition_variable cv_{};
  std::mutex sync_mtx_{};
  std::mutex task_done_m_{};
  std::condition_variable task_done_cv_{};
  bool task_done_{false};
  // async task
  std::mutex async_mtx_{};
  std::vector<std::function<void()>> async_tasks_{};

  // thread
  std::future<void> fut_;
  std::atomic<bool> canceled_{false};
  
 void do_sync_task(std::function<void()> fun){
   std::unique_lock<std::mutex> lock_sync(sync_mtx_);
   sync_task_ = fun;
   {
     std::unique_lock<std::mutex> lock2(cv_m_);
     cv_.notify_one();
     while (!task_done_){
       task_done_cv_.wait(lock2);
     }
   }
   task_done_ = false;
  }

  void run_async_tasks(){
    std::vector<std::function<void()>> tasks_to_do{};
    { std::unique_lock<std::mutex> lock_sync(async_mtx_);
      auto num_tasks = async_tasks_.size();
      if (0 != num_tasks){
        tasks_to_do.resize(num_tasks);
        std::move(async_tasks_.begin(),
                  async_tasks_.end(),
                  tasks_to_do.begin());
        async_tasks_.clear();
      }
    }
    for (auto &it: tasks_to_do)
      it();
    tasks_to_do.clear();
  }
};

}  // namespace switcher
#endif
