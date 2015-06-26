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
#include "shmdata/clogger.h"
#include "shmdata/cwriter.h"

int num_successive_write = 5;

void mylog(void *user_data, const char *str) {
  printf("%s\n", str);
}

int main () {
  ShmdataLogger logger = shmdata_make_logger(&mylog,
                                             &mylog,
                                             &mylog,
                                             &mylog,
                                             &mylog,
                                             &mylog,
                                             NULL);
  assert(NULL != logger);
  int i = num_successive_write;
  while (-1 != --i) {
    ShmdataWriter writer = shmdata_make_writer("/tmp/check-shmdatasrc",
                                               sizeof(int),
                                               "application/x-check-shmdata",
                                               NULL,
                                               NULL,
                                               NULL,
                                               logger);
    assert(NULL != writer); 
    usleep(50000);
    int j = 10;
    while (-1 != --j) {
      // first option: copy
      assert(0 != shmdata_copy_to_shm(writer, &j, sizeof(int)));
    }
    shmdata_delete_writer(writer);
    usleep(50000);
    }
  shmdata_delete_logger(logger);
  return 0;
}

