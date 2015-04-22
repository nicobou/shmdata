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

#include "cfollower.h"
#include <string>
#include "./follower.hpp"

namespace shmdata{
class CFollower {
 public:
  CFollower(const char *path,
            void (*on_data_cb)(void *user_data,
                               void *data,
                               size_t size),
            void (*on_server_connected)(void *user_data,
                                        const char *type_descr),
            void (*on_server_disconnected)(void *user_data),
            void *user_data,
            ShmdataLogger log) :
      on_data_cb_(on_data_cb),
      on_server_connected_(on_server_connected),
      on_server_disconnected_(on_server_disconnected),
      user_data_(user_data),
    follower_(path,
              [&](void *data, size_t size){
                if(nullptr != on_data_cb_)
                  on_data_cb_(user_data_, data, size);
              },
              [&](const std::string &type_descr){
                if(nullptr != on_server_connected_)
                  on_server_connected_(user_data_, type_descr.c_str());
              },
              [&](){
                if(nullptr != on_server_disconnected_)
                  on_server_disconnected_(user_data_);
              },
              static_cast<AbstractLogger *>(log)){
  }
 private:
  void (*on_data_cb_)(void *user_data, void *data, size_t size);
  void (*on_server_connected_)(void *user_data, const char *type_descr);
  void (*on_server_disconnected_)(void *user_data);
  void *user_data_;
  Follower follower_;
};
}  // namespace shmdata

ShmdataFollower shmdata_make_follower(const char *path,
                                      void (*on_data_cb)(void *user_data,
                                                         void *data,
                                                         size_t size),
                                      void(*on_server_connected)(void *user_data,
                                                                 const char *type_descr),
                                      void(*on_server_disconnected)(void *user_data),
                                      void *user_data,
                                      ShmdataLogger log){
  return static_cast<void *>(new shmdata::CFollower(path,
                                                    on_data_cb,
                                                    on_server_connected,
                                                    on_server_disconnected,
                                                    user_data,
                                                    log));
}
void shmdata_delete_follower(ShmdataFollower follower){
  delete static_cast<shmdata::CFollower *>(follower);
}
