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

namespace semops{
// sem_num 0 is for reader, 1 is for writer, 2 is for locking until first writer
static struct sembuf read_start [] = {{0,1,SEM_UNDO},      // incr reader
                                      {1,0,SEM_UNDO}};     // wait 0 on writer
static struct sembuf read_end [] = {{0,-1,SEM_UNDO}};      // decr reader
static struct sembuf write_start1 [] = {{1,1,SEM_UNDO}};   // incr writer
static struct sembuf write_start2 [] = {{0,0,SEM_UNDO},    // wait reader is 0
                                        {0,1,SEM_UNDO}};   // incr reader
static struct sembuf write_fail_end [] = {{1,-1,SEM_UNDO}};// decr writer
static struct sembuf write_end [] = {{0,-1,SEM_UNDO},      // decr reader
                                     {1,-1,SEM_UNDO}};     // decr writer
}  // namespace semops

sysVSem::sysVSem(key_t key, int semflg) :
    key_ (key),
    semid_(semget(key_, 3, semflg)) {
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

readLock::readLock(sysVSem *sem) :
    semid_(sem->semid_) {
  if (-1 == semop(semid_,
                  semops::read_start,
                  sizeof(semops::read_start)/sizeof(*semops::read_start))){
    valid_ = false;  // TODO log this
  }
}

readLock::~readLock(){
  if (is_valid())
    semop(semid_,
          semops::read_end,
          sizeof(semops::read_end)/sizeof(*semops::read_end));
}

writeLock::writeLock(sysVSem *sem) :
    semid_(sem->semid_){
  if (-1 == semop(semid_,
                  semops::write_start1,
                  sizeof(semops::write_start1)/sizeof(*semops::write_start1))) {
    valid_ = false;
    return;
  }
  if (-1 == semop(semid_,
                  semops::write_start2,
                  sizeof(semops::write_start2)/sizeof(*semops::write_start2))) {
    write_fail_ = true;
    return;
  }

}

writeLock::~writeLock(){
  if(!is_valid())
    return;
  if(write_fail_) {
     semop(semid_,
           semops::write_fail_end,
           sizeof(semops::write_fail_end)/sizeof(*semops::write_fail_end));
  } else {
    semop(semid_,
          semops::write_end,
          sizeof(semops::write_end)/sizeof(*semops::write_end));
  }
}

}  // namespace shmdata
