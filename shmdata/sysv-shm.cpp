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
#include <string.h>        // memset
#include <stdio.h>         // fopen 
#include "./sysv-shm.hpp"

#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

#if HAVE_OSX
#include <sys/types.h>
#include <sys/sysctl.h>
#else
#ifndef SHMMAX_SYS_FILE
#define SHMMAX_SYS_FILE "/proc/sys/kernel/shmmax"
#endif
#ifndef SHMMNI_SYS_FILE
#define SHMMNI_SYS_FILE "/proc/sys/kernel/shmmni"
#endif
#endif  // HAVE_OSX

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
  
unsigned long sysVShm::get_shmmax(AbstractLogger *log){
#if HAVE_OSX
  unsigned long shmmax = 0;
  size_t len = sizeof(shmmax);
  if (-1 == sysctlbyname("kern.sysv.shmmax", &shmmax, &len, NULL, 0)){
    int err = errno;
    log->error("trying to get kern.sysv.shmmax: ", strerror(err));
  }
  return shmmax;
#else
  unsigned long shmmax = 0;
  FILE *shmmax_file = fopen(SHMMAX_SYS_FILE, "r");
  if (!shmmax_file) {
    if (nullptr != log)
      //FIXME errno
      log->error("Failed to open shmmax from file:" SHMMAX_SYS_FILE);
    return 0;
  }
  if (fscanf(shmmax_file, "%lu", &shmmax) != 1) {
    if (nullptr != log)
      // FIXME errno
      log->error("Failed to read shmmax from file:" SHMMAX_SYS_FILE);
    fclose(shmmax_file);
    return 0;
  }
  fclose(shmmax_file);
  return shmmax;
#endif  // HAVE_OSX
}

unsigned long sysVShm::get_shmmni(AbstractLogger *log){
#if HAVE_OSX
  unsigned long shmmni = 0;
  size_t len = sizeof(shmmni);
  if (-1 == sysctlbyname("kern.sysv.shmmni", &shmmni, &len, NULL, 0)){
    int err = errno;
    log->error("trying to get kern.sysv.shmmni: ", strerror(err));
  }
  return shmmni;
#else
  unsigned long shmmni = 0;
  FILE *shmmni_file = fopen(SHMMNI_SYS_FILE, "r");
  if (!shmmni_file) {
    if (nullptr != log)
      //FIXME errno
      log->error("Failed to open shmmni from file:" SHMMNI_SYS_FILE);
    return 0;
  }
  if (fscanf(shmmni_file, "%lu", &shmmni) != 1) {
    if (nullptr != log)
      // FIXME errno
      log->error("Failed to read shmmni from file:" SHMMNI_SYS_FILE);
    fclose(shmmni_file);
    return 0;
  }
  fclose(shmmni_file);
  return shmmni;
#endif  // HAVE_OSX
}

  
}  // namespace shmdata
