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

#include "./sysv-sem.hpp"
#include <errno.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/types.h>

namespace shmdata {

bool force_semaphore_cleaning(key_t key, AbstractLogger* log) {
  auto semid = semget(key, 2, 0);
  if (semid < 0) {
    int err = errno;
    log->debug("semget (forcing semaphore cleaning): %", strerror(err));
    return false;
  }
  if (semctl(semid, 0, IPC_RMID, 0) != 0) {
    int err = errno;
    log->error("semctl removing semaphore %", strerror(err));
  }
  return true;
}

namespace semops {
// sem_num 0 is for reading, 1 is for writer
static struct sembuf read_start[] = {{1, 0, 0}};   // wait writer
static struct sembuf read_end[] = {{0, -1, 0}};    // decr reader
static struct sembuf write_start[] = {{0, 0, 0},   // wait reader is 0
                                      {1, 1, 0},   // incr writer
                                      {0, 1, 0}};  // incr reader
static struct sembuf write_end[] = {{0, -1, 0},    // decr reader
                                    {1, -1, 0}};   // decr writer
}  // namespace semops

sysVSem::sysVSem(key_t key, AbstractLogger* log, bool owner, mode_t unix_permission)
    : key_(key),
      owner_(owner),
      semid_(semget(key_, 2, owner ? IPC_CREAT | IPC_EXCL | unix_permission : 0)),
      log_(log) {
  if (semid_ < 0) {
    int err = errno;
    log_->debug("semget: %", strerror(err));
    return;
  }
}

sysVSem::~sysVSem() {
  if (is_valid() && owner_) {
    if (semctl(semid_, 0, IPC_RMID, 0) != 0) {
      int err = errno;
      log_->error("semctl removing semaphore %", strerror(err));
    }
  }
}

void sysVSem::cancel_commited_reader() {
  if (-1 == semop(semid_, semops::read_end, sizeof(semops::read_end) / sizeof(*semops::read_end))) {
    int err = errno;
    log_->error("semop cancel: %", strerror(err));
  }
}

bool sysVSem::is_valid() const { return 0 < semid_; }

ReadLock::ReadLock(sysVSem* sem) : sem_(sem) {
  if (-1 == semop(sem_->semid_,
                  semops::read_start,
                  sizeof(semops::read_start) / sizeof(*semops::read_start))) {
    int err = errno;
    sem_->log_->debug("semop ReadLock %", strerror(err));
    valid_ = false;
  }
}

ReadLock::~ReadLock() {
  if (is_valid())
    semop(sem_->semid_, semops::read_end, sizeof(semops::read_end) / sizeof(*semops::read_end));
}

WriteLock::WriteLock(sysVSem* sem) : sem_(sem) {
  if (-1 == semop(sem_->semid_,
                  semops::write_start,
                  sizeof(semops::write_start) / sizeof(*semops::write_start))) {
    int err = errno;
    sem_->log_->error("semop WriteLock: %", strerror(err));
    valid_ = false;
  }
}

bool WriteLock::commit_readers(short num_reader) {
  struct sembuf read_commit_reader[] = {{0, num_reader, 0}};
  if (-1 == semop(sem_->semid_,
                  read_commit_reader,
                  sizeof(read_commit_reader) / sizeof(*read_commit_reader))) {
    int err = errno;
    sem_->log_->error("semop commit readers: %", strerror(err));
    return false;
  }
  return true;
}
WriteLock::~WriteLock() {
  if (!is_valid()) return;
  semop(sem_->semid_, semops::write_end, sizeof(semops::write_end) / sizeof(*semops::write_end));
}

}  // namespace shmdata
