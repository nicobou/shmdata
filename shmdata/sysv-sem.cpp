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
#include <sys/ipc.h>
#include <sys/sem.h>
#include <stdio.h>  // perror
#include "./sysv-sem.hpp"


namespace shmdata{

sysVSem::sysVSem(key_t key, int semflg) :
    key_ (key),
    semid_(semget(key_, 1, semflg)) {
  if (semid_ < 0) {
    perror("semget");
    return;
  }
}

sysVSem::~sysVSem() {
  if (is_valid()) {
    if (semctl(semid_, 0, IPC_RMID, 0) != 0) {
      perror("semctl removing semaphore");
    }
  }
}

bool sysVSem::is_valid() const {
  return 0 < semid_;
}

}  // namespace shmdata
