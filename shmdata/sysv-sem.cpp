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
#include <iostream>
#include "./sysv-sem.hpp"


namespace shmdata{

namespace semops{
// sem_num 0 is for reading, 1 is for writer
static struct sembuf read_start [] = {{1, 0, SEM_UNDO}};     // wait writer
static struct sembuf read_end [] = {{0, -1, SEM_UNDO}};       // decr reader
static struct sembuf write_start [] = {{0, 0, SEM_UNDO},     // wait reader is 0
                                       {1, 1, SEM_UNDO},    // incr writer
                                       {0, 1, SEM_UNDO}};   // incr reader
static struct sembuf write_end [] = {{0, -1, SEM_UNDO},     // decr reader
                                     {1, -1, SEM_UNDO}};    // decr writer
}  // namespace semops

sysVSem::sysVSem(key_t key, bool owner) :
    key_ (key),
    owner_(owner),
    semid_(semget(key_, 2, owner ? IPC_CREAT | IPC_EXCL | 0666 : 0)) {
  if (semid_ < 0) {
    perror("semget");
    return;
  }
}

sysVSem::~sysVSem() {
  if (is_valid() && owner_) {
    if (semctl(semid_, 0, IPC_RMID, 0) != 0) {
      perror("semctl removing semaphore");
    }
  }
}

void sysVSem::cancel_commited_reader(){
  if (-1 == semop(semid_,
                  semops::read_end,
                  sizeof(semops::read_end)/sizeof(*semops::read_end))){
    perror("semop");
    std::cout << "bug cancel commited reader" << std::endl;
  }
}

bool sysVSem::is_valid() const {
  return 0 < semid_;
}

ReadLock::ReadLock(sysVSem *sem) :
    semid_(sem->semid_) {
  if (-1 == semop(semid_,
                  semops::read_start,
                  sizeof(semops::read_start)/sizeof(*semops::read_start))){
    valid_ = false;  // TODO log this
  }
}

ReadLock::~ReadLock(){
  if (is_valid())
    semop(semid_,
          semops::read_end,
          sizeof(semops::read_end)/sizeof(*semops::read_end));
}

WriteLock::WriteLock(sysVSem *sem) :
    semid_(sem->semid_) {
  if (-1 == semop(semid_,
                  semops::write_start,
                  sizeof(semops::write_start)/sizeof(*semops::write_start))) {
    valid_ = false;
  }
}

bool WriteLock::commit_readers(short num_reader){
  struct sembuf read_commit_reader [] = {{0, num_reader, SEM_UNDO}};
  if (-1 == semop(semid_,
                  read_commit_reader,
                  sizeof(read_commit_reader)/sizeof(*read_commit_reader))) {
    return false;
  }
  return true;
}
WriteLock::~WriteLock(){
  if(!is_valid())
    return;
  semop(semid_,
        semops::write_end,
        sizeof(semops::write_end)/sizeof(*semops::write_end));
}

}  // namespace shmdata
