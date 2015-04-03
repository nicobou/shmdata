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
#include <thread>  //yield
#include "./sysv-sem.hpp"


namespace shmdata{

namespace semops{
// sem_num 0 is for reading, 1 is for writer, 2 is for data available, 3 for going to read
static struct sembuf sem_init [] = {{2, 1, 0}};
static struct sembuf read_wait [] = {{2, 0, 0},      // wait data
                                     {3, 1, 0}};     // incr going to read
static struct sembuf read_start [] = {{0, 1, 0},      // incr reader
                                      {1, 0, 0},      // wait writer
                                      {3, -1, 0}};    // decr going to read
static struct sembuf read_end [] = {{0, -1, 0}};      // decr reader
static struct sembuf write_start2 [] = {{0, 0, 0},    // wait reader is 0
                                        {1, 1, 0},    // incr writer
                                        {0, 1, 0},    // incr reader
                                        {2, -1, 0}};  // updating data
static struct sembuf write_end1 [] = {{0, -1, 0},      // decr reader
                                     {1, -1, 0},      // decr writer
                                     {2, 1, 0}};      // end updating data
static struct sembuf write_end2 [] = {{3, 0, 0}};   // wait going to read
}  // namespace semops

sysVSem::sysVSem(key_t key, int semflg) :
    key_ (key),
    semid_(semget(key_, 4, semflg)) {
  if (semid_ < 0) {
    perror("semget");
    return;
  }
  if (-1 == semop(semid_,
                  semops::sem_init,
                  sizeof(semops::sem_init)/sizeof(*semops::sem_init))) {
    perror("sem init");
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
                  semops::read_wait,
                  sizeof(semops::read_wait)/sizeof(*semops::read_wait))){
    valid_ = false;  // TODO log this
  }
  if (-1 == semop(semid_,
                  semops::read_start,
                  sizeof(semops::read_start)/sizeof(*semops::read_start))){
    valid_ = false;  // TODO log this
  }
  std::this_thread::yield();
}

readLock::~readLock(){
  if (is_valid())
    semop(semid_,
          semops::read_end,
          sizeof(semops::read_end)/sizeof(*semops::read_end));
  std::this_thread::yield();
}

writeLock::writeLock(sysVSem *sem) :
    semid_(sem->semid_){
  if (-1 == semop(semid_,
                  semops::write_start2,
                  sizeof(semops::write_start2)/sizeof(*semops::write_start2))) {
    valid_ = false;
    return;
  }
  std::this_thread::yield();
}

writeLock::~writeLock(){
  if(!is_valid())
    return;
  semop(semid_,
        semops::write_end1,
        sizeof(semops::write_end1)/sizeof(*semops::write_end1));
  semop(semid_,
        semops::write_end2,
        sizeof(semops::write_end2)/sizeof(*semops::write_end2));
  std::this_thread::yield();
}

}  // namespace shmdata
