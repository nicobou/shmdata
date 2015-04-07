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

#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>  // std::cout
#include "./file-monitor.hpp"

namespace shmdata{
namespace fileMonitor{

bool is_unix_socket(const std::string &path){
  struct stat sb;
  if (stat(path.c_str(), &sb) == -1) {
    //perror("stat");
    return false;
  }
  if ((sb.st_mode & S_IFMT) != S_IFSOCK) {
    std::cout << "file type is not a socket" << std::endl;
    return false;
  }
  return true;
}

}  // namespace fileMonitor
}  // namespace shmdata
