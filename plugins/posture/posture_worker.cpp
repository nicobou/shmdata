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
