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

#include <iostream>  // debug
#include "./reader.hpp"

namespace shmdata{

Reader::Reader(const std::string &path) :
    path_(path),
    proto_([this](int){this->on_server_connected();},
           [this](int){this->on_server_disconnected();}),
    cli_(path, &proto_),
    shm_(ftok(path.c_str(), 'n'), 0, /* owner = */ false),
    sem_(ftok(path.c_str(), 'm'), /* owner = */ false) {
  if (!cli_ || !shm_ || !sem_)
    is_valid_ = false;
}

void Reader::on_server_connected(){
  std::cout << "(client) on_connect_cb "
            << " shm_size " << proto_.data_.shm_size_ 
            << " user_data " << proto_.data_.get_user_data().c_str()
            << std::endl;
}

void Reader::on_server_disconnected(){
  std::cout << "(client) on_disconnect_cb "
            << std::endl;
}


}  // namespace shmdata
