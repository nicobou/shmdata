/*
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

#ifndef _SHMDATA_C_FOLLOWER_H_
#define _SHMDATA_C_FOLLOWER_H_

#include <stdlib.h>
#include "./clogger.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup  capi C API
 * The port of the C++ API in C.
 *
 * Here is a usage example for the C API, that illustrate the writing and reading of a shmdata
 * from a same provcess: \include tests/check-c-wrapper.cpp
 *  @{
 */
typedef void* ShmdataFollower;

/**
 * \brief Construct of a ShmdataFollower that read a shmdata, and handle
 * connection/disconnection of the writer.
 * Information and data are provided asynchronously by the Follower though callbacks.
 *
 * \param   path                     Shmdata path to follow
 * \param   on_data_cb               Callback to be triggered when a frame is published
 * \param   on_server_connected      Callback to be triggered when the follower
 *                                   connected with the shmdata writer
 * \param   on_server_disconnected   Callback to be triggered when the follower
 *                                   disconnected from the shmdata writer
 * \param   user_data                Pointer to be given back
 *                                   when the callback is triggered
 * \param   log                      Log object where to write
 *                                   internal logs
 *
 * \return  Created ShmdataFollower
 */
ShmdataFollower shmdata_make_follower(const char* path,
                                      void (*on_data_cb)(void* user_data, void* data, size_t size),
                                      void (*on_server_connected)(void* user_data,
                                                                  const char* type_descr),
                                      void (*on_server_disconnected)(void* user_data),
                                      void* user_data,
                                      ShmdataLogger log);

/**
 * \brief Delete a ShmdataFollower and release associated ressources
 */
void shmdata_delete_follower(ShmdataFollower follower);

/** @}*/
#ifdef __cplusplus
}
#endif

#endif
