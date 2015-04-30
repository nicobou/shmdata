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
#include "./writer.hpp"

namespace shmdata{

Writer::Writer(const std::string &path,
               size_t memsize,
               const std::string &data_descr,
               AbstractLogger *log,
               UnixSocketProtocol::ServerSide::onClientConnect on_client_connect,
               UnixSocketProtocol::ServerSide::onClientDisconnect on_client_disconnect):
    connect_data_(memsize, data_descr),
    proto_(on_client_connect,
           on_client_disconnect,
           [this](){return this->connect_data_;}),
    srv_(path, &proto_, log, [&](int){sem_.cancel_commited_reader();}),
    shm_(ftok(path.c_str(), 'n'), memsize, log, /*owner = */ true),
    sem_(ftok(path.c_str(), 'm'), log, /*owner = */ true),
    log_(log) {
  if (!srv_ || !shm_ || !sem_) {
    log_->error("writer failled to initialize");
    is_valid_ = false;
  }
  log_->debug("writer initialized");
}

bool Writer::copy_to_shm(void *data, size_t size){
  bool res = true;
  {
    if (size > connect_data_.shm_size_)
      return false;
    WriteLock wlock(&sem_);
    auto num_readers = srv_.notify_update(size);
    if (0 < num_readers) {
      wlock.commit_readers(num_readers);
    }
    auto dest = shm_.get_mem();
    if ( dest != std::memcpy(dest, data, size))
      res = false;
  } // release wlock & lock
  return res;
}

std::unique_ptr<OneWriteAccess> Writer::get_one_write_access(size_t size) {
  return std::unique_ptr<OneWriteAccess>(new OneWriteAccess(&sem_,
                                                            shm_.get_mem(),
                                                            &srv_,
                                                            size,
                                                            log_));
}

OneWriteAccess *Writer::get_one_write_access_ptr(size_t size) {
  return new OneWriteAccess(&sem_,
                            shm_.get_mem(),
                            &srv_,
                            size,
                            log_);
}

OneWriteAccess::OneWriteAccess(sysVSem *sem,
                               void *mem,
                               UnixSocketServer *srv,
                               size_t size,
                               AbstractLogger *log) :
    wlock_(sem),
    mem_(mem),
    srv_(srv),
    size_(size),
    log_(log){
}

short OneWriteAccess::notify_clients(){
  if (has_notified_) {
    log_->warning("one notification only is expected per OneWriteAccess instance, "
                  "ignoring current invocation");
    return 0;
  }
  has_notified_ = true;
  short num_readers = srv_->notify_update(size_);
  //log->debug("one write access for % readers", std::to_string(num_readers));
  if (0 < num_readers) {
    wlock_.commit_readers(num_readers);
  }
  return num_readers;
}

}  // namespace shmdata
