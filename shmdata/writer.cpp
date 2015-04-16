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

#include <cstring>  // memcpy
#include <iostream> // debug
#include "./writer.hpp"

namespace shmdata{

Writer::Writer(const std::string &path, size_t memsize, const std::string &data_descr):
    connect_data_(memsize, data_descr),
    proto_([](int){},
           [](int){},
           [this](){return this->connect_data_;}),
    srv_(path, &proto_, [&](int){sem_.cancel_commited_reader();}),
    shm_(ftok(path.c_str(), 'n'), memsize, /*owner = */ true),
    sem_(ftok(path.c_str(), 'm'), /*owner = */ true)
{
  if (!srv_ || !shm_ || !sem_)
      is_valid_ = false;
}

bool Writer::copy_to_shm(void *data, size_t size){
  bool res = true;
  {
    if (size > connect_data_.shm_size_)
      return false;
    WriteLock wlock(&sem_);
    auto num_readers = srv_.notify_update();
    if (0 < num_readers) {
      wlock.commit_readers(num_readers);
    }
    auto dest = shm_.get_mem();
    if ( dest != std::memcpy(dest, data, size))
      res = false;
  } // release wlock & lock
  return res;
}

std::unique_ptr<OneWriteAccess> Writer::get_one_write_access() {
  return std::unique_ptr<OneWriteAccess>(new OneWriteAccess(&sem_,
                                                            shm_.get_mem(),
                                                            &srv_));
}

OneWriteAccess::OneWriteAccess(sysVSem *sem,
                               void *mem,
                               UnixSocketServer *srv) :
    wlock_(sem),
    mem_(mem){
  auto num_readers = srv->notify_update();
  if (0 < num_readers) {
    wlock_.commit_readers(num_readers);
  }
}

}  // namespace shmdata
