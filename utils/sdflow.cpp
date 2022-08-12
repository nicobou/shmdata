/*
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
#include <unistd.h>

#include <chrono>
#include <climits>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <thread>

#include "shmdata/console-logger.hpp"
#include "shmdata/follower.hpp"

#define MAX_FRAME_TIME_COUNT 30

using namespace shmdata;

static std::unique_ptr<Follower> follower;

void leave(int sig) {
  follower.reset(nullptr);
  exit(sig);
}

void usage(const char *prog_name){
  printf("usage: %s [-d] [-f] [-v] [-c] shmpath\n", prog_name);
  exit(1);
}

int main (int argc, char *argv[]) {
  bool debug = false;
  bool show_frame_timings = false;
  bool show_version = false;
  bool cartridge_return = false;
  char *shmpath = nullptr;

  opterr = 0;
  int c = 0;
  while ((c = getopt (argc, argv, "cdfv")) != -1)
    switch (c)
      {
        case 'c':
          cartridge_return = true;
          break;
        case 'd':
          debug = true;
          break;
        case 'f':
          show_frame_timings = true;
          break;
        case 'v':
          show_version = true;
          break;
        case '?':
          break;
        default:
          usage(argv[0]);
      }

  if (show_version) {
    std::printf("%s\n", SHMDATA_VERSION_STRING);
    exit(1);
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
    std::deque<int64_t> frames_time;

    follower.reset(
        new Follower(shmpath,
                     [&frame_count, &frames_time, show_frame_timings, cartridge_return](void *data, size_t size){
                       float frame_duration = 0.f;
                       float framerate = 0.f;
                       if (show_frame_timings) {
                         int64_t current_frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                         frames_time.push_back(current_frame_time);

                         while (frames_time.size() > MAX_FRAME_TIME_COUNT)
                           frames_time.pop_front();

                         if (frames_time.size() > 1) {
                           frame_duration = frames_time.back() - frames_time[frames_time.size() - 2];
                           int64_t total_frames_duration = frames_time.back() - frames_time.front();
                           float mean_time_per_frame = static_cast<float>(total_frames_duration) / static_cast<float>(frames_time.size() - 1);
                           framerate = 1000.f / mean_time_per_frame;
                         }
                       }

                       if(cartridge_return)
                         std::cout << '\r';
                       std::cout << frame_count;

                       if (show_frame_timings) {
                       std::cout << std::setprecision(0) << std::fixed << "    dT: " << frame_duration << "ms"
                                 << std::setprecision(2) << std::fixed << "    fps: " << framerate;
                       }

                       std::cout << "    size: " << size
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
                       if (cartridge_return)
                         std::cout <<  std::flush;
                       else
                         std::cout <<  std::endl;
                     },
                     [&](const std::string &str){
                       std::cout << "connected: type " << str << std::endl;
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

