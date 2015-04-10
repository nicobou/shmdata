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

class WriteLock;
class ReadLock;
class sysVSem: public SafeBoolIdiom {
  friend WriteLock;
  friend ReadLock;
 public:
  explicit sysVSem(key_t key, bool owner = false);
  ~sysVSem();
  sysVSem() = delete;
  sysVSem(const sysVSem &) = delete;
  sysVSem& operator=(const sysVSem&) = delete;
  sysVSem& operator=(sysVSem&&) = default;
  
 private:
  key_t key_;
  bool owner_;
  int semid_{-1};
  bool is_valid() const final;
};

class ReadLock: public SafeBoolIdiom {
 public:
  ReadLock(sysVSem *sem); 
  ~ReadLock();
  ReadLock() = delete;
  ReadLock(const ReadLock &) = delete;
  ReadLock& operator=(const ReadLock&) = delete;
  ReadLock& operator=(ReadLock&&) = delete;

  //void set_last(){is_last_ = true;}
 private:
  int semid_;
  bool valid_{true};
  //bool is_last_{false};
  bool is_valid() const final {return valid_;};
};

class WriteLock: public SafeBoolIdiom {
 public:
  WriteLock(sysVSem *sem);
  ~WriteLock();
  WriteLock() = delete;
  WriteLock(const WriteLock &) = delete;
  WriteLock& operator=(const WriteLock&) = delete;
  WriteLock& operator=(WriteLock&&) = default;
 private:
  int semid_;
  bool valid_{true};
  bool is_valid() const final {return valid_;};
};

}  // namespace shmdata
#endif
