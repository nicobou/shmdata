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

#include "./file-utils.hpp"
#include <dirent.h>
#include <fcntl.h>
#include <gio/gio.h>  // GFile
#include <glib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <utility>
#include "./scope-exit.hpp"
#include "./string-utils.hpp"

namespace switcher {

std::pair<bool, std::string> FileUtils::prepare_writable_dir(const std::string& path) {
  if (is_dir(path)) {
    if (0 != access(path.c_str(), W_OK | X_OK)) {  // not writable
      int err = errno;
      return std::make_pair(false, std::string(strerror(err)));
    }
    return std::make_pair(true, std::string());
  }

  // trying to create the directory
  std::size_t i = 0;
  while (std::string::npos != i) {
    std::size_t found = path.find('/', i);
    if (i != found) {
      auto dir = std::string(path, 0, found);
      if (!is_dir(dir)) {
        auto res = create_writable_dir(dir);
        if (!res.first) return res;
      }
    }
    if (std::string::npos != found) {
      i = found + 1;
    } else {
      i = std::string::npos;
    }
  }
  return std::make_pair(true, std::string());
}

bool FileUtils::is_dir(const std::string& path) {
  struct stat sb;
  if (stat(path.c_str(), &sb) == -1)  // does not exists
    return false;
  if ((sb.st_mode & S_IFMT) != S_IFDIR)  // is not a directory
    return false;
  return true;
}

std::pair<bool, std::string> FileUtils::create_writable_dir(const std::string& path) {
  if (-1 == mkdir(path.c_str(), S_IRWXU)) {
    int err = errno;
    return std::make_pair(false, std::string(strerror(err)));
  }
  return std::make_pair(true, std::string());
}

std::vector<std::string> FileUtils::get_files_from_directory(std::string path,
                                                             std::string prefix,
                                                             std::string suffix,
                                                             bool recursive) {
  DIR* directory;
  struct dirent* entry;
  std::vector<std::string> files;

  directory = opendir(path.c_str());
  if (!directory) return files;
  On_scope_exit { closedir(directory); };

  while ((entry = readdir(directory))) {
    if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") continue;

    if (entry->d_type == DT_REG) {
      if ((prefix.empty() || StringUtils::starts_with(entry->d_name, prefix)) &&
          (suffix.empty() || StringUtils::ends_with(entry->d_name, suffix))) {
        files.push_back(path + "/" + entry->d_name);
      }
    } else if (entry->d_type == DT_DIR && recursive) {
      auto inner_folder_files =
          get_files_from_directory(path + "/" + entry->d_name, prefix, suffix, recursive);
      files.insert(files.end(), inner_folder_files.begin(), inner_folder_files.end());
    }
  }

  return files;
}

std::pair<std::string, std::string> FileUtils::get_file_content(const std::string& file_path,
                                                                int max_file_size) {
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    return std::make_pair(std::string(), std::string("cannot open ") + file_path);
  }
  // get file content into a string
  std::string res;
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    return std::make_pair(std::string(), std::string("file ") + file_path + " is empty");
  }
  if (size > max_file_size) {
    return std::make_pair(std::string(),
                          std::string("file ") + file_path + " is too large, max is " +
                              std::to_string(max_file_size) + " bytes");
  }
  res.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  res.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  return std::make_pair(res, std::string());
}

BoolLog FileUtils::save(const std::string& content, const std::string& file_path) {
  if (content.empty()) {
    return BoolLog(false, "cannot save empty content to file");
  }
  GFile* file = g_file_new_for_commandline_arg(file_path.c_str());
  On_scope_exit { g_object_unref(file); };
  GError* error = nullptr;
  GFileOutputStream* file_stream = g_file_replace(file,
                                                  nullptr,
                                                  TRUE,  // make backup
                                                  G_FILE_CREATE_NONE,
                                                  nullptr,
                                                  &error);
  On_scope_exit { g_object_unref(file_stream); };
  if (error != nullptr) {
    On_scope_exit { g_error_free(error); };
    return BoolLog(false, error->message);
  }
  // saving the save tree to the file
  gchar* history = g_strdup(content.c_str());
  g_output_stream_write(
      (GOutputStream*)file_stream, history, sizeof(gchar) * strlen(history), nullptr, &error);
  g_free(history);
  if (error != nullptr) {
    On_scope_exit { g_error_free(error); };
    return BoolLog(false, error->message);
  }
  g_output_stream_close((GOutputStream*)file_stream, nullptr, &error);
  if (error != nullptr) {
    On_scope_exit { g_error_free(error); };
    return BoolLog(false, error->message);
  }
  return true;
}

std::string FileUtils::get_content(const std::string& file_path) {
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    return std::string();
  }
  // get file content into a string
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    return std::string();
  }
  std::string file_str;
  file_str.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  file_str.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  return file_str;
}

}  // namespace switcher
