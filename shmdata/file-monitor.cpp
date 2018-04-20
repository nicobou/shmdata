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

#include "./file-monitor.hpp"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

namespace shmdata {
namespace fileMonitor {

bool is_unix_socket(const std::string& path, AbstractLogger* log) {
  struct stat sb;
  if (stat(path.c_str(), &sb) == -1) {
    // int err = errno;
    // log->debug("stat %", strerror(err));
    return false;
  }
  if ((sb.st_mode & S_IFMT) != S_IFSOCK) {
    log->error("% is not a socket", path);
    return false;
  }
  return true;
}

}  // namespace fileMonitor
}  // namespace shmdata
