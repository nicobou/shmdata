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

static bool debug = false;
static bool verbose = false;
static bool properly_quit = false;
static int num_writes = 0;
static bool num_writes_enabled = false;
static ShmdataLogger logger = nullptr;
static ShmdataWriter writer = nullptr;

// a struct with contiguous data storage 
typedef struct _Frame {
  size_t count = 0;
} Frame;

void usage(const char *prog_name){
  printf("create a shmdata writer, write to it, and do not clean when dying\n"
         "usage: %s  [-d] [-v] [-q] [-n num_writes] shmpath\n"
         "  -d enable debug\n"
         "  -v enable verbose\n"
         "  -q enable quitting properly\n"
         "  -n num enable quitting after num write(s)\n"
         , prog_name);
  exit(1);
}

void leave(int sig) {
  if (properly_quit){
    shmdata_delete_writer(writer);
    shmdata_delete_logger(logger);
  }
  exit(sig);
}

void mylog(void */*user_data*/, const char *str) {
  if (debug)
    printf("%s\n", str);
}

int main (int argc, char *argv[]) {
  (void) signal(SIGINT, leave);
  (void) signal(SIGABRT, leave);
  (void) signal(SIGQUIT, leave);
  (void) signal(SIGTERM, leave);

  char *shmpath = nullptr;
  opterr = 0;
  int c = 0;
  while ((c = getopt (argc, argv, "dvqn:")) != -1)
    switch (c)
      {
        case 'd':
          debug = true;
          break;
        case 'v':
          verbose = true;
          break;
        case 'q':
          properly_quit = true;
          break;
        case 'n':
          num_writes = atoi(optarg);
          num_writes_enabled = true;
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

  logger = shmdata_make_logger(&mylog, &mylog, &mylog, &mylog, &mylog, &mylog, NULL);
  writer = shmdata_make_writer(shmpath,
                               sizeof(Frame),
                               "application/x-sdcrash",
                               NULL,
                               NULL,
                               NULL,
                               logger);
  if (!writer ){
    printf("writer failled: quit without trying to write\n");
    return 0;
  }
    
  Frame frame;
  frame.count = 0;
  bool quit = false;
  while (!quit) {
    ++frame.count;
    if (num_writes_enabled && num_writes <= 0) {
      quit = true;
      continue;
    } 
    usleep(50000);
    if (verbose) printf("w-");
    shmdata_copy_to_shm(writer, &frame, sizeof(Frame));
    if (verbose) printf("done\n");
    if (num_writes_enabled) num_writes --;
  }  // end while
  if (properly_quit){
    shmdata_delete_writer(writer);
    shmdata_delete_logger(logger);
  }
  return 0;
}

