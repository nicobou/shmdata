/*
 * Copyright (C) 2015 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */


#ifndef _SHMDATA_SYSV_SEM_H_
#define _SHMDATA_SYSV_SEM_H_

#include <sys/ipc.h>
#include <sys/sem.h>
#include "./safe-bool-idiom.hpp"

namespace shmdata{

class writeLock;
class updateSubscriber;
class sysVSem: public SafeBoolIdiom {
  friend writeLock;
  friend updateSubscriber;
 public:
  sysVSem(key_t key, int semflg);
  ~sysVSem();
  sysVSem() = delete;
  sysVSem(const sysVSem &) = delete;
  sysVSem& operator=(const sysVSem&) = delete;
  sysVSem& operator=(sysVSem&&) = default;
  
 private:
  key_t key_;
  int semid_{-1};
  bool is_valid() const final;
};

// subscribing actually at first read
class readLock;
class updateSubscriber {
  friend readLock;
 public:
  updateSubscriber(sysVSem *sem);
  ~updateSubscriber();
  updateSubscriber() = delete;
  updateSubscriber(const updateSubscriber &) = delete;
  updateSubscriber& operator=(const updateSubscriber&) = delete;
  updateSubscriber& operator=(updateSubscriber&&) = delete;

  void stop() {is_stoped_ = true;};
 private:
  int semid_;
  bool do_wait_{true};
  bool is_stoped_{false};
};

class readLock: public SafeBoolIdiom {
 public:
  // if is_single_read is false, readLock is locking
  // writer for avoiding the next read missing an update  
  readLock(updateSubscriber *subscriber); 
  ~readLock();
  readLock() = delete;
  readLock(const readLock &) = delete;
  readLock& operator=(const readLock&) = delete;
  readLock& operator=(readLock&&) = delete;

  //void set_last(){is_last_ = true;}
 private:
  updateSubscriber *sub_;
  bool valid_{true};
  //bool is_last_{false};
  bool is_valid() const final {return valid_;};
};

class writeLock: public SafeBoolIdiom {
 public:
  writeLock(sysVSem *sem);
  ~writeLock();
  writeLock() = delete;
  writeLock(const writeLock &) = delete;
  writeLock& operator=(const writeLock&) = delete;
  writeLock& operator=(writeLock&&) = default;
 private:
  int semid_;
  bool valid_{true};
  bool is_valid() const final {return valid_;};
};

}  // namespace shmdata
#endif
