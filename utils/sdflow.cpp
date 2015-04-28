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

#include <signal.h>
#include <memory>
#include <iostream>
#include <thread>
#include <algorithm>
#include "shmdata/follower.hpp"
#include "shmdata/console-logger.hpp"

using namespace shmdata;

static std::unique_ptr<Follower> follower;

void leave(int sig) {
  follower.reset(nullptr);
  exit(sig);
}


int main (int argc, char *argv[]) {

  if (argc != 2) {
    
  }
  (void) signal(SIGINT, leave);
  (void) signal(SIGABRT, leave);
  (void) signal(SIGQUIT, leave);
  (void) signal(SIGTERM, leave);

  using namespace shmdata;
  ConsoleLogger logger;
  logger.set_debug(false);
  {
    follower.reset(
        new Follower("/tmp/blah",
                     [](void *data, size_t size){
                       std::cout << "ptr "
                                 << data
                                 << " size " << size
                                 << "  data extract: ";
                       auto end = 4u;
                       short *vals = static_cast<short *>(data);
                       if (end * sizeof(short) > size)
                         end = size;
                       while (0 != end) { 
                         std::printf("%02x", vals[end]);
                         --end;
                       }
                       std::cout << std::endl;
                     },
                     [&](const std::string &str){
                       std::cout << "connected type " << str <<std::endl;
                     },
                     [&](){std::cout << "disconnected" << std::endl;},
                     &logger));
    // wait
    while(true){
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  return 0;
}

