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

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "shmdata/cwriter.h"
#include "shmdata/clogger.h"

static bool debug = true;

// a struct with contiguous data storage 
typedef struct _Frame {
  size_t count = 0;
} Frame;

void usage(const char *prog_name){
  printf("usage: %s  [-d] shmpath\n", prog_name);
  exit(1);
}

void mylog(void */*user_data*/, const char *str) {
  printf("%s\n", str);
}

int main (int argc, char *argv[]) {
  char *shmpath = nullptr;

  opterr = 0;
  int c = 0;
  while ((c = getopt (argc, argv, "d:")) != -1)
    switch (c)
      {
        case 'd':
          debug = true;
          shmpath = optarg;
          break;
        case '?':
          break;
        default:
          usage(argv[0]);
      }

  if (nullptr == shmpath && optind + 1 == argc)
    shmpath = argv[optind];
  if (nullptr == shmpath)
    usage(argv[0]);

  ShmdataLogger logger =
      shmdata_make_logger(&mylog, &mylog, &mylog, &mylog, &mylog, &mylog, NULL);
  ShmdataWriter writer = shmdata_make_writer(shmpath,
                                             sizeof(Frame),
                                             "application/x-sdcrash",
                                             NULL,
                                             NULL,
                                             NULL,
                                             logger);
  if (!writer){
    printf("quit without trying to write\n");
    return 0;
  }
    
  Frame frame;
  frame.count = 0;
  while (true) {
    ++frame.count;
    shmdata_copy_to_shm(writer, &frame, sizeof(Frame));
  usleep(50000);
  }
  
  return 0;
}

