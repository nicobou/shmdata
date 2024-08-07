/*
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

#ifndef _SHMDATA_SYSV_SHM_H_
#define _SHMDATA_SYSV_SHM_H_

#include <sys/ipc.h>
#include <sys/shm.h>
#include <string>
#include "./abstract-logger.hpp"
#include "./safe-bool-idiom.hpp"

namespace shmdata {

bool force_shm_cleaning(key_t key, AbstractLogger* log);

class sysVShm : public SafeBoolIdiom {
 public:
  sysVShm(key_t key,
          size_t size,
          AbstractLogger* log,
          bool owner = false,
          mode_t unix_permission = 0600);
  ~sysVShm();
  sysVShm() = delete;
  sysVShm(const sysVShm&) = delete;
  sysVShm& operator=(const sysVShm&) = delete;
  sysVShm& operator=(sysVShm&&) = default;

  void* get_mem() { return shm_; }
  std::size_t get_size() const { return size_; }
  // Maximum size in bytes for a shared memory segment
  static unsigned long get_shmmax(AbstractLogger* log);
  // System-wide limit on the number of shared memory segments
  static unsigned long get_shmmni(AbstractLogger* log);

 private:
  AbstractLogger* log_;
  key_t key_;
  size_t size_;
  int shmid_;
  bool owner_;            // responsible for creation and deletion of the shm
  void* shm_{(void*)-1};  // man shmat
  bool is_valid() const final;
};

}  // namespace shmdata
#endif
