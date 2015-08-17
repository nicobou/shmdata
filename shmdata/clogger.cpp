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

#include "clogger.h"
#include "./abstract-logger.hpp"

namespace shmdata{

class CLogger: public AbstractLogger {
 public:
  CLogger(void (*on_error)(void *user_data, const char *),
          void (*on_critical)(void *user_data, const char *),
          void (*on_warning)(void *user_data, const char *),
          void (*on_message)(void *user_data, const char *),
          void (*on_info)(void *user_data, const char *),
          void (*on_debug)(void *user_data, const char *),
          void *user_data):
      on_error_(on_error),
      on_critical_(on_critical),
      on_warning_(on_warning),
      on_message_(on_message),
      on_info_(on_info),
      on_debug_(on_debug),
      user_data_(user_data){
  }
  virtual ~CLogger(){};
  
 private:
  void (*on_error_)(void *user_data, const char *);
  void (*on_critical_)(void *user_data, const char *);
  void (*on_warning_)(void *user_data, const char *);
  void (*on_message_)(void *user_data, const char *);
  void (*on_info_)(void *user_data, const char *);
  void (*on_debug_)(void *user_data, const char *);
  void *user_data_;
  
  void on_error(std::string &&str) final {
    if (nullptr != on_error_) on_error_(user_data_, str.c_str());
  }
  void on_critical(std::string &&str) final {
    if (nullptr != on_critical_) on_critical_(user_data_, str.c_str());
  }
  void on_warning(std::string &&str) final {
    if (nullptr != on_warning_) on_warning_(user_data_, str.c_str());
  }
  void on_message(std::string &&str) final {
    if (nullptr != on_message_) on_message_(user_data_, str.c_str());
  }
  void on_info(std::string &&str) final {
    if (nullptr != on_info_) on_info_(user_data_, str.c_str());
  }
  void on_debug(std::string &&str) final {
    if (nullptr != on_debug_) on_debug_(user_data_, str.c_str());
  }
};

}  // namespace shmdata

ShmdataLogger shmdata_make_logger(void (*on_error)(void *user_data, const char *),
                                  void (*on_critical)(void *user_data, const char *),
                                  void (*on_warning)(void *user_data, const char *),
                                  void (*on_message)(void *user_data, const char *),
                                  void (*on_info)(void *user_data, const char *),
                                  void (*on_debug)(void *user_data, const char *),
                                  void *user_data){
  return static_cast<void *>(
      new shmdata::CLogger(on_error,
                           on_critical,
                           on_warning,
                           on_message,
                           on_info,
                           on_debug,
                           user_data));
}

void shmdata_delete_logger(ShmdataLogger logger){
  delete static_cast<shmdata::CLogger *>(logger);
}

