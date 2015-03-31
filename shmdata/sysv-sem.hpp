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


#ifndef _SHMDATA_SYSV_SEM_H_
#define _SHMDATA_SYSV_SEM_H_

#include <sys/types.h>
#include "./safe-bool-idiom.hpp"

namespace shmdata{

class sysVSem: public SafeBoolIdiom {
 public:
  sysVSem(key_t key, int semflg);
  ~sysVSem();
  sysVSem() = delete;
  sysVSem(const sysVSem &) = delete;
  sysVSem& operator=(const sysVSem&) = delete;
  sysVSem& operator=(sysVSem&&) = default;
  
 private:
  key_t key_;
  int semid_{-1};
  bool is_valid() const final;
};

}  // namespace shmdata
#endif
