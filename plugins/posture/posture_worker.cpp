#include "posture_worker.hpp"

#include <iostream>

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
    ready_ = false;
    task_ = make_shared<function<void()>>(func);
    _condition.notify_one();
  }
  
  /*************/
  void Worker::thread_loop()
  {
    while (true)
    {
      unique_lock<mutex> lock(mutex_);
      ready_ = true;
      _condition.wait(lock);

      if (task_ == nullptr)
        continue;
  
      if (stop_)
        return;
  
      (*task_)();
      task_.reset();
    }
  }

} // end of namespace
