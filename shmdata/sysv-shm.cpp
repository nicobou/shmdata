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
#include <stdio.h>  // perror
#include <string.h>  // memset
#include "./sysv-shm.hpp"

namespace shmdata{

sysVShm::sysVShm(key_t key, size_t size, int shmflg):
    key_(key),
    size_(size),
    shmid_(shmget(key_, size_, shmflg)) {
  if (-1 == key_){
    perror("ftok");
    return;
  }
  if (shmid_ < 0){
      perror("shmget");
      return;
  }
  if ((shm_ = shmat(shmid_, NULL, 0)) == (void *) -1) {
    perror("shmat");  
    return;
  }
  memset(shm_, 0, size_);
}

sysVShm::~sysVShm() {
  if (is_valid()) {
    if (shmdt(shm_) == -1) {
      perror("shmdt");
    }
  }
  if (0 < shmid_) {
    if (shmctl(shmid_, IPC_RMID, NULL) < 0)
      perror("shmctl removing shared mem");
  }
}

bool sysVShm::is_valid() const {
  return (shm_ != (void *) -1);
}

}  // namespace shmdata
