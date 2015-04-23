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

#ifndef _CONSOLE_C_LOGGER_H_
#define _CONSOLE_C_LOGGER_H_

#ifdef __cplusplus
extern "C" {
#endif
  
  typedef void *ShmdataLogger;
  ShmdataLogger shmdata_make_logger(void (*on_error)(void *user_data, const char *),
                                    void (*on_critical)(void *user_data, const char *),
                                    void (*on_warning)(void *user_data, const char *),
                                    void (*on_message)(void *user_data, const char *),
                                    void (*on_info)(void *user_data, const char *),
                                    void (*on_debug)(void *user_data, const char *),
                                    void *user_data);
  void shmdata_delete_logger(ShmdataLogger logger);
  
#ifdef __cplusplus
}
#endif

#endif
