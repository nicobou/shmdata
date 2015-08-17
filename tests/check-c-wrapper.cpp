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

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <exception>
#include "shmdata/clogger.h"
#include "shmdata/cfollower.h"
#include "shmdata/cwriter.h"

static char my_user_data[6] = "hello";
int server_interactions = 0;
int num_successive_write = 5;

// a struct with contiguous data storage 
typedef struct _Frame {
  size_t count = 0;
  const int tab[3] = {1, 2, 3};
} Frame;

void mylog(void *user_data, const char *str) {
  assert(user_data == my_user_data);
  printf("%s\n", str);
}

void on_server_connect(void *user_data, const char *type_descr) {
  assert(user_data == my_user_data);
  assert(0 == strcmp(type_descr, "application/x-check-shmdata"));
  ++server_interactions;
}

void on_server_disconnect(void *user_data) {
  assert(user_data == my_user_data);
  ++server_interactions;
}

void on_data(void *user_data, void *data, size_t size) {
  assert(user_data == my_user_data);
  Frame *frame = (Frame *)data;
  printf("new data for client: %zu size %zu ptr %p\n",
         frame->count,
         size,
         data);
}

int main () {
  ShmdataLogger logger = shmdata_make_logger(&mylog,
                                             &mylog,
                                             &mylog,
                                             &mylog,
                                             &mylog,
                                             &mylog,
                                             my_user_data);
  assert(NULL != logger);
  ShmdataFollower follower = shmdata_make_follower("/tmp/check-c-wrapper",
                                                   &on_data,
                                                   &on_server_connect,
                                                   &on_server_disconnect,
                                                   my_user_data,
                                                   logger);
  assert(NULL != follower);
  int i = num_successive_write;
  while (-1 != --i) {
    ShmdataWriter writer = shmdata_make_writer("/tmp/check-c-wrapper",
                                               sizeof(Frame),
                                               "application/x-check-shmdata",
                                               NULL,
                                               NULL,
                                               NULL,
                                               logger);
    assert(NULL != writer); 
    usleep(50000);
    Frame frame;
    frame.count = 0;
    int j = 10;
    while (-1 != --j) {
      // first option: copy
      ++frame.count;
      assert(0 != shmdata_copy_to_shm(writer, &frame, sizeof(Frame)));
      // second option: direct write
      ShmdataWriterAccess access = shmdata_get_one_write_access(writer);
      Frame *shared_frame = (Frame *)shmdata_get_mem(access);
      shared_frame->count = ++frame.count;
      shmdata_notify_clients(access, sizeof(Frame));
      shmdata_release_one_write_access(access);
      }
    shmdata_delete_writer(writer);
    usleep(50000);
    }

  shmdata_delete_follower(follower);
  shmdata_delete_logger(logger);

  
  if (server_interactions == 2 * num_successive_write)
     return 0;
  return 1;  // fail
}

