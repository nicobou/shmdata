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


#ifndef _SHMDATA_FILE_MONITOR_H_
#define _SHMDATA_FILE_MONITOR_H_

#include <string>
#include "./abstract-logger.hpp"

namespace shmdata{
namespace fileMonitor{

bool is_unix_socket(const std::string &path,
                    AbstractLogger *log);

}  // namespace fileMonitor
}  // namespace shmdata
#endif
