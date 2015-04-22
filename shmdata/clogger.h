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

typedef void *ShmdataCLogger;
extern "C" ShmdataCLogger Shmdata_make_logger(void (*on_error)(const char *),
                                              void (*on_critical)(const char *),
                                              void (*on_warning)(const char *),
                                              void (*on_massage)(const char *),
                                              void (*on_info)(const char *),
                                              void (*on_debug)(const char *));
extern "C" void Shmdata_delete_logger(ShmdataCLogger logger);

#endif
