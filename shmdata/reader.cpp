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

#include "./reader.hpp"

namespace shmdata{

Reader::Reader(const std::string &path,
               onData cb,
               onServerConnected osc,
               onServerDisconnected osd,
               AbstractLogger *log) :
    log_(log),
    path_(path),
    on_data_cb_(cb),
    on_server_connected_cb_(osc),
    on_server_disconnected_cb_(osd),
    proto_([this](){on_server_connected();},
           [this](){on_server_disconnected();},
           [this](size_t size){on_buffer(this->sem_.get(), size);}),  // read when update is received
    cli_(new UnixSocketClient(path, log_)){
  if (!cli_ || !(*cli_.get())) {
    //log_->debug("reader initialization failed (initializing socket client)");
    cli_.reset(nullptr);
  
    return;
  }
  shm_.reset(new sysVShm(ftok(path.c_str(), 'n'), 0, log_, /* owner = */ false));
  sem_.reset(new sysVSem(ftok(path.c_str(), 'm'), log_, /* owner = */ false));
  if (!*shm_.get() || !*sem_.get() || !cli_->start(&proto_)){
    //log_->debug("reader initialization failed");
    cli_.reset();
    shm_.reset();
    sem_.reset();
    return;
  }
  is_valid_ = true;
  log_->debug("reader initialization done");
}

void Reader::on_server_connected(){
  log_->debug("received server info, shm_size %, type %",
                std::to_string(proto_.data_.shm_size_),
                proto_.data_.user_data_.data());
  if (on_server_connected_cb_)
    on_server_connected_cb_(proto_.data_.user_data_.data());
}

void Reader::on_server_disconnected(){
  log_->debug("disconnected from server");
  if (on_server_disconnected_cb_)
    on_server_disconnected_cb_();
}

bool Reader::on_buffer(sysVSem *sem, size_t size){
  ReadLock lock(sem);
  if (!lock) 
    return false;
  if (on_data_cb_)
    on_data_cb_(shm_->get_mem(), size);
  return true;
}

}  // namespace shmdata
