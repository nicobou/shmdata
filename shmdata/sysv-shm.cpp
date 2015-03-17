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
#include "./sysv-shm.hpp"

namespace shmdata{

sysVShm::sysVShm(key_t key, size_t size, int shmflg):
    key_(key),
    size_(size),
    shmflg_(shmflg) {
  if ((shmid_ = shmget(key_, size_, shmflg_)) < 0){
      perror("shmget");  // TODO get string of perror and log info
      return;
  }
  if ((shm_ = shmat(shmid_, NULL, 0)) == (void *) -1) {
    perror("shmat");  
    return;
  }
}

sysVShm::~sysVShm(){
  if (shm_ == (void *) -1) { // fixme use safe bool
    if (shmdt(shm_) == -1) {
      perror("shmdt");
    }
  }
}

}  // namespace shmdata
