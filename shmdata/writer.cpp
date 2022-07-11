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

#include "./writer.hpp"
#include <cstring>  // memcpy
#include "./reader.hpp"

namespace shmdata {

Writer::Writer(const std::string& path,
               size_t memsize,
               const std::string& data_descr,
               AbstractLogger* log,
               UnixSocketProtocol::ServerSide::onClientConnect on_client_connect,
               UnixSocketProtocol::ServerSide::onClientDisconnect on_client_disconnect,
               mode_t unix_permission)
    : path_(path),
      connect_data_(memsize, data_descr),
      proto_(on_client_connect, on_client_disconnect, [this]() { return this->connect_data_; }),
      srv_(new UnixSocketServer(path, &proto_, log, [&](int) { sem_->cancel_commited_reader(); }, unix_permission)),
      shm_(new sysVShm(ftok(path.c_str(), 'n'),
                       memsize,
                       log,
                       /*owner = */ true,
                       unix_permission)),
      sem_(new sysVSem(ftok(path.c_str(), 'm'), log, /*owner = */ true, unix_permission)),
      log_(log),
      alloc_size_(memsize) {
  if (!(*srv_.get()) || !(*shm_.get()) || !(*sem_.get())) {
    sem_.reset();
    shm_.reset();
    srv_.reset();
    // checking if a server is responding with at this shmdata path
    bool can_read = false;
    {
      log_->debug("checking if a shmdata having the same path is already active");
      Reader inspector(path, nullptr, nullptr, nullptr, log_);
      can_read = static_cast<bool>(inspector);
    }
    if (!can_read) {
      log_->debug("writer detected a dead Shmdata, will clean and retry");
      force_sockserv_cleaning(path, log);
      srv_.reset(
          new UnixSocketServer(path, &proto_, log, [&](int) { sem_->cancel_commited_reader(); }, unix_permission));
      force_shm_cleaning(ftok(path.c_str(), 'n'), log);
      shm_.reset(new sysVShm(ftok(path.c_str(), 'n'),
                             memsize,
                             log,
                             /*owner = */ true,
                             unix_permission));
      force_semaphore_cleaning(ftok(path.c_str(), 'm'), log);
      sem_.reset(new sysVSem(ftok(path.c_str(), 'm'), log, /*owner = */ true, unix_permission));
      is_valid_ = (*srv_.get()) && (*shm_.get()) && (*sem_.get());
    } else {
      log_->error("an other writer is using the same path");
      is_valid_ = false;
    }
  }
  if (!is_valid_) {
    log_->warning("writer failled initialization");
    return;
  }
  srv_->start_serving();
  log_->debug("writer initialized");
}

bool Writer::copy_to_shm(const void* data, size_t size) {
  bool res = true;
  {
    if (nullptr == sem_) {
      log_->warning("semaphore is not initialized");
      return false;
    }

    if (!(*sem_.get())) {
      log_->warning("semaphore was not correctly initialized");
      return false;
    }
    WriteLock wlock(sem_.get());
    if (size > connect_data_.shm_size_) {
      log_->debug("resizing shmdata (%) from % bytes to % bytes",
                  path_,
                  std::to_string(connect_data_.shm_size_),
                  std::to_string(size));
      shm_.reset();
      shm_.reset(new sysVShm(ftok(path_.c_str(), 'n'), size, log_, /*owner = */ true));
      connect_data_.shm_size_ = size;
      alloc_size_ = size;
      if (!shm_) {
        log_->error("resizing shared memory failed");
        return false;
      }
      
    }
    auto num_readers = srv_->notify_update(size);
    if (0 < num_readers) {
      wlock.commit_readers(num_readers);
    }
    auto dest = shm_->get_mem();
    if (dest != std::memcpy(dest, data, size)) res = false;
  }  // release wlock & lock
  return res;
}

std::unique_ptr<OneWriteAccess> Writer::get_one_write_access() {
  return std::unique_ptr<OneWriteAccess>(
      new OneWriteAccess(this, sem_.get(), shm_->get_mem(), srv_.get(), log_));
}

OneWriteAccess* Writer::get_one_write_access_ptr() {
  return new OneWriteAccess(this, sem_.get(), shm_->get_mem(), srv_.get(), log_);
}

std::unique_ptr<OneWriteAccess> Writer::get_one_write_access_resize(size_t new_size) {
  auto res = std::unique_ptr<OneWriteAccess>(
      new OneWriteAccess(this, sem_.get(), nullptr, srv_.get(), log_));
  if (shm_->get_size() != new_size) {
    log_->debug("resizing shmdata (%) from % bytes to % bytes",
                path_,
                std::to_string(connect_data_.shm_size_),
                std::to_string(new_size));
    shm_.reset();
    shm_.reset(new sysVShm(ftok(path_.c_str(), 'n'), new_size, log_, /*owner = */ true));
  }
  res->mem_ = shm_->get_mem();
  connect_data_.shm_size_ = new_size;
  alloc_size_ = new_size;
  return res;
}

OneWriteAccess* Writer::get_one_write_access_ptr_resize(size_t new_size) {
  auto res = new OneWriteAccess(this, sem_.get(), nullptr, srv_.get(), log_);
  log_->debug("resizing shmdata (%) from % bytes to % bytes",
              path_,
              std::to_string(connect_data_.shm_size_),
              std::to_string(new_size));
  shm_.reset();
  shm_.reset(new sysVShm(ftok(path_.c_str(), 'n'), new_size, log_, /*owner = */ true));
  res->mem_ = shm_->get_mem();
  connect_data_.shm_size_ = new_size;
  alloc_size_ = new_size;
  return res;
}

size_t Writer::alloc_size() const { return alloc_size_; }

OneWriteAccess::OneWriteAccess(
    Writer* writer, sysVSem* sem, void* mem, UnixSocketServer* srv, AbstractLogger* log)
    : writer_(writer), wlock_(sem), mem_(mem), srv_(srv), log_(log) {}

size_t OneWriteAccess::shm_resize(size_t new_size) {
  writer_->shm_.reset();
  writer_->shm_.reset(
      new sysVShm(ftok(writer_->path_.c_str(), 'n'), new_size, log_, /*owner = */ true));
  if (!writer_->shm_) return 0;
  mem_ = writer_->shm_->get_mem();
  writer_->connect_data_.shm_size_ = new_size;
  writer_->alloc_size_ = new_size;
  return new_size;
}

short OneWriteAccess::notify_clients(size_t size) {
  if (has_notified_) {
    log_->warning(
        "one notification only is expected per OneWriteAccess instance, "
        "ignoring current invocation");
    return 0;
  }
  has_notified_ = true;
  short num_readers = srv_->notify_update(size);
  // log->debug("one write access for % readers", std::to_string(num_readers));
  if (0 < num_readers) {
    wlock_.commit_readers(num_readers);
  }
  return num_readers;
}

}  // namespace shmdata
