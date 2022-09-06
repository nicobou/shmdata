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

template <typename T>
void print_vals(const void *data, unsigned int max_val_displayed, std::size_t size, const std::string& printf_format) {
  auto vals = static_cast<const T *>(data);
  auto val_index = 0u;
  while (val_index < max_val_displayed && val_index * sizeof(T) < size) { 
    std::printf(printf_format.c_str(), vals[val_index]);
    ++val_index;
  }
  if (val_index != size / sizeof(T)) std::printf("...");
}

template <>
void print_vals<char>(const void *data, unsigned int max_val_displayed, std::size_t size, const std::string& printf_format) {
  auto vals = static_cast<const char *>(data);
  auto val_index = 0u;
  while (val_index < max_val_displayed && val_index * sizeof(char) < size) { 
    std::printf(printf_format.c_str(), vals[val_index] & 0xff);
    ++val_index;
  }
  if (val_index != size / sizeof(char)) std::printf("...");
}

void leave(int sig) {
  follower.reset(nullptr);
  exit(sig);
}

void usage(const char *prog_name){
  printf("usage: %s [OPTIONS] shmpath\n", prog_name);
  printf(R""""(
sdflow prints data from an existing Shmdata.
It aims at providing a debugging tool with the Shmdata library.

OPTIONS:
  -f format  print values using 'format',
               'x'   for hexadecimal
               'f'   for float
               'i16' for integer 16bits
               'c'   for char
  -m num     print first 'num' values for each frame (default is 12)
  -c         print shmdata type and then information about last frame (carriage return)
  -t         print frame timing information (delay and frame-per-seconds)
  -n         print data only (no frame number, no size and no timing information)
  -d         print debug option
  -v         print Shmdata version and exits

)"""");
  exit(1);
}

int main (int argc, char *argv[]) {
  bool debug = false;
  bool show_frame_timings = false;
  bool show_version = false;
  bool cartridge_return = false;
  auto max_val_displayed = 12u;
  char *shmpath = nullptr;
  std::string format = "x";
  bool print_info = true;
  
  opterr = 0;
  int c = 0;
  while ((c = getopt (argc, argv, "cdf:tvm:n")) != -1)
    switch (c)
      {
        case 'c':
          cartridge_return = true;
          break;
        case 'd':
          debug = true;
          break;
        case 'f':
          format = std::string(optarg);
          break;
        case 't':
          show_frame_timings = true;
          break;
        case 'm':
          max_val_displayed = static_cast<unsigned int>(atoi(optarg));
          break;
        case 'n':
          print_info = false;
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
                     [&](void *data, size_t size){
                       if(cartridge_return)
                         std::cout << "\33[2K\r";
                       if (print_info) {
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
                         
                         // print informations
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
                       } // end print_info
                       if (format == "f") {
                         print_vals<float>(data, max_val_displayed, size, "% 1.2f ");
                       } else if (format == "c") {
                         print_vals<char>(data, max_val_displayed, size, "%c");
                       } else if (format == "i16") {
                         print_vals<int16_t>(data, max_val_displayed, size, "% 05d ");
                       } else {
                         print_vals<char>(data, max_val_displayed, size, "%02x ");
                       }
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
