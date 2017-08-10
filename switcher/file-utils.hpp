/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef __SWITCHER_FILE_UTILS_H__
#define __SWITCHER_FILE_UTILS_H__

#include <string>
#include <utility>
#include <vector>
#include "./bool-log.hpp"

namespace switcher {
namespace FileUtils {

// A message is given in case of error
std::pair<bool, std::string> prepare_writable_dir(const std::string& path);

// A message is given in case of error
// all subdirs must exist
std::pair<bool, std::string> create_writable_dir(const std::string& path);

bool is_dir(const std::string& path);

std::vector<std::string> get_files_from_directory(std::string path,
                                                  std::string prefix = "",
                                                  std::string suffix = "",
                                                  bool recursive = false);

// default is 100Mo
std::pair<std::string, std::string> get_file_content(const std::string& file_path,
                                                     int max_file_size = 100000000);

BoolLog save(const std::string& content, const std::string& file_path);
std::string get_content(const std::string& file_path);

}  // namespace FileUtils
}  // namespace switcher
#endif
