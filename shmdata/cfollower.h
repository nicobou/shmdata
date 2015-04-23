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

#ifndef _SHMDATA_C_FOLLOWER_H_
#define _SHMDATA_C_FOLLOWER_H_

#include <stdlib.h>
#include "./clogger.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef void * ShmdataFollower;

  ShmdataFollower shmdata_make_follower(
      const char *path,
      void (*on_data_cb)(void *user_data,
                         void *data,
                         size_t size),
      void(*on_server_connected)(void *user_data,
                                 const char *type_descr),
      void(*on_server_disconnected)(void *user_data),
      void *user_data,
      ShmdataLogger log);
  void shmdata_delete_follower(ShmdataFollower follower);

#ifdef __cplusplus
}
#endif

#endif
