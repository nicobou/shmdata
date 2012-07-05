/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

/** \addtogroup libshmdata
 * provides hot plugging between GStreamer pipelines via a shared memory.
 * compile with `pkg-config --cflags --libs shmdata-0.3`
 *  @{
 */

#ifndef _SHMDATA_BASE_READER_H_
#define _SHMDATA_BASE_READER_H_

#include <gst/gst.h>
#include <gio/gio.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @file   base-reader.h
   * 
   * @brief  Reading from a shared memory and forward to a GStreamer pipeline
   * 
   * The base reader provides a shared memory reader, allowing forwarding 
   * of a data flow to a GStreamer pipeline. It supports hot connections 
   * and disconnections of the writer. The reader is monitoring the socket 
   * path for automatically connect to the shared memory when created.
   * 
   */


  typedef struct shmdata_base_reader_ shmdata_base_reader_t; 

  /*! \fn void (*on_first_data)(shmdata_base_reader_t *reader, void *userdata);
   *  \brief Callback triggered when the writer is connecting. In this function 
   *  the sink must be set with shmdata_base_reader_set_sink. 
   *  \param reader is the reader 
   *  \param user_data is a pointer to the user data 
   */
  typedef void (*shmdata_base_reader_on_first_data)(shmdata_base_reader_t *reader, void *user_data);

  /** 
   * Initialization function that starts monitoring the socket path.
   * 
   * @param socketPath is the file name of the shared memory
   * @param Pipeline is the pipeline where the base writer will be added
   * @param on_first_data is the function pointer that will be called when 
   * the connecting with the writer.
   * @param user_data is the user data pointer for the on_first_video_data
   * callback
   * 
   * @return a base reader
   */
  shmdata_base_reader_t *shmdata_base_reader_init (const char *socketPath,
						   GstElement * Pipeline,
						   shmdata_base_reader_on_first_data cb,
						   void *user_data);

  /** 
   * Tell the reader in which GStreamer element the reader should push the 
   * data. This function should be called in the on_first_data handler.
   * 
   * @param reader is the base reader to inform
   * @param sink is the element the base reader must link with in order to
   * transmit data. This element is assumed to be in the same pipeline as the 
   * reader.
   *
   */

  void shmdata_base_reader_set_sink (shmdata_base_reader_t * reader,
				     GstElement * sink);

  /** 
   * Close the reader (freeing memory and removing internal GStreamer elements 
   * from the pipeline). 
   * 
   * @param reader is the base reader to close
   */
  void shmdata_base_reader_close (shmdata_base_reader_t * reader);

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHMDATA_BASE_READER_H_

/** @}*/
