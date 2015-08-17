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

#include <unistd.h>
#include <signal.h>
#include <memory>
#include <iostream>
#include <thread>
#include <algorithm>
#include <climits>
#include "shmdata/follower.hpp"
#include "shmdata/console-logger.hpp"

using namespace shmdata;

static std::unique_ptr<Follower> follower;

void leave(int sig) {
  follower.reset(nullptr);
  exit(sig);
}

void usage(const char *prog_name){
  printf("usage: %s [-d] shmpath\n", prog_name);
  exit(1);
}

int main (int argc, char *argv[]) {
  bool debug = false;
  char *shmpath = nullptr;

  opterr = 0;
  int c = 0;
  while ((c = getopt (argc, argv, "d")) != -1)
    switch (c)
      {
        case 'd':
          debug = true;
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

  (void) signal(SIGINT, leave);
  (void) signal(SIGABRT, leave);
  (void) signal(SIGQUIT, leave);
  (void) signal(SIGTERM, leave);
  
  using namespace shmdata;
  ConsoleLogger logger;
  logger.set_debug(debug);
  unsigned long long int frame_count = 0;
  {
    follower.reset(
        new Follower(shmpath,
                     [&frame_count](void *data, size_t size){
                       std::cout << frame_count 
                                 << "    size: " << size
                                 << "    data: ";
                       if (ULLONG_MAX == frame_count)
                         frame_count = 0;
                       else
                         ++frame_count;
                       std::string etc;
                       auto end = 15u;
                       char *vals = static_cast<char *>(data);
                       if (end * sizeof(char) > size) {
                         end = size;
                       } else
                         etc = "...";
                       while (0 != end) { 
                         std::printf("%02X", vals[end] & 0xff );
                         --end;
                       }
                       if (!etc.empty())
                         std::cout << etc;
                       std::cout <<  std::endl;
                     },
                     [&](const std::string &str){
                       std::cout << "connected: type " << str <<std::endl;
                     },
                     [&](){
                       std::cout << "disconnected" << std::endl;
                     },
                     &logger));
    // wait
    while(true){
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  return 0;
}

