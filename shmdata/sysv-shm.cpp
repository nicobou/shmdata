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

#include <sys/types.h>
#include <errno.h>
#include <string.h>  // memset
#include "./sysv-shm.hpp"

namespace shmdata{

bool force_shm_cleaning(key_t key, AbstractLogger *log){
  auto shmid = shmget(key, 0, 0);
  if (shmid < 0) {
    int err = errno;
    log->debug("shmget (forcing shm cleaning): %", strerror(err));
    return false;
  }
  if (shmctl(shmid, IPC_RMID, NULL) != 0) {
    int err = errno;
    log->error("shmctl removing shm: %", strerror(err));
  }
  return true;
}

sysVShm::sysVShm(key_t key, size_t size, AbstractLogger *log, bool owner):
    log_(log),
    key_(key),
    size_(size),
    shmid_(shmget(key_, size_, owner ? (IPC_CREAT | IPC_EXCL | 0666) : 0)),
    owner_(owner) {
  if (-1 == key_){
    int err = errno;
    log_->warning("ftok: %", strerror(err));
    return;
  }
  if (shmid_ < 0){
    int err = errno;
    log_->warning("shmget: %", strerror(err));
    return;
  }
  if ((shm_ = shmat(shmid_, NULL, 0)) == (void *) -1) {
    int err = errno;
    log_->warning("shmat: %", strerror(err));
    return;
  }
  memset(shm_, 0, size_);
}

sysVShm::~sysVShm() {
  if (is_valid()) {
    if (shmdt(shm_) == -1) {
      int err = errno;
      log_->error("shmdt: %", strerror(err));
    }
  }
  if (0 < shmid_ && owner_) {
    if (shmctl(shmid_, IPC_RMID, NULL) < 0) {
      int err = errno;
      log_->error("shmctl removing shared mem: %", strerror(err));
    }
  }
}

bool sysVShm::is_valid() const {
  return (-1 != key_) && (shmid_ >= 0) && (shm_ != (void *) -1);
}

}  // namespace shmdata
