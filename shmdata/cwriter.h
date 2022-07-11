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

#ifndef _SHMDATA_C_WRITER_H_
#define _SHMDATA_C_WRITER_H_

#include <stdlib.h>
#include "./clogger.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef void * ShmdataWriter;
  typedef void * ShmdataWriterAccess;

  /**
   * \brief Construct a ShmdataWriter.
   *
   * \param   path                  Shmdata path for listening incomming connections by Followers.
   * \param   memsize               Initial size of the shared memory.
   *                                Note the shared memory can be resized at each frame.
   * \param   type_desr             A string description for the frame to be transmitted.
   *                                It is expected to follow syntax of GStreamer caps.  
   * \param   on_client_connect     Callback to be triggered when a follower connects.
   * \param   on_client_disconnect  Callback to be triggered when a follower disconnects.
   * \param   user_data             Pointer to user data passed to connect and disconnect callbacks.
   * \param   log                   Log object where to write internal logs.
   * \param   unix_permission       Permission to apply to the internal Unix socket, shared memory and semaphore.
   *
   * \return the created ShmdataWriter
   */

  ShmdataWriter shmdata_make_writer(const char* path,
                                    size_t memsize,
                                    const char* type_descr,
                                    void (*on_client_connected)(void* user_data, int id),
                                    void (*on_client_disconnected)(void* user_data, int id),
                                    void* user_data,
                                    ShmdataLogger log,
                                    mode_t unix_permission);

  /**
   * \brief Delete the ShmdataWriter and release associated ressources.
   *
   */
  void shmdata_delete_writer(ShmdataWriter writer);

  // write copying data
  int shmdata_copy_to_shm(ShmdataWriter writer,
                          const void *data,
                          size_t size);

  // or get write lock and notify clients when they can try locking for reading 
  ShmdataWriterAccess shmdata_get_one_write_access(ShmdataWriter writer);
  ShmdataWriterAccess shmdata_get_one_write_access_resize(ShmdataWriter writer, size_t newsize);
  // this will allocate a new uninitialized shared memory space  
  size_t shmdata_shm_resize(ShmdataWriterAccess access, size_t new_size);
  void *shmdata_get_mem(ShmdataWriterAccess access);
  short shmdata_notify_clients(ShmdataWriterAccess access, size_t size);
  void shmdata_release_one_write_access(ShmdataWriterAccess access);

  // Maximum size in bytes for a shared memory segment
  unsigned long shmdata_get_shmmax(ShmdataLogger log);
  // System-wide limit on the number of shared memory segments
  unsigned long shmdata_get_shmmni(ShmdataLogger log);

#ifdef __cplusplus
}
#endif

#endif
