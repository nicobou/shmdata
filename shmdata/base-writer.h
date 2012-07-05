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
 * compile with `pkg-config --cflags --libs shmdata-0.4`
 *  @{
 */


#ifndef _SHMDATA_BASE_WRITER_H_
#define _SHMDATA_BASE_WRITER_H_

#include <gst/gst.h>
#include <gio/gio.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef SHMDATA_OK
#define SHMDATA_OK 1
#endif
#ifndef SHMDATA_FILE_EXISTS
#define SHMDATA_FILE_EXISTS 0
#endif

  /**
   * @file   base-writer.h
   * 
   * @brief  Writing to a shared memory from a GStreamer element or pad
   * 
   * The base writer provides a shared memory writer, allowing transmission 
   * of data flows to other GStreamer pipelines. It supports hot connections 
   * and disconnections of multiple reader. A single writer is allowed for 
   * a given shared memory (socket path).
   *
   */


  typedef struct shmdata_base_writer_ shmdata_base_writer_t;

  /** 
   * Initialization function that creates the shared memory.
   * 
   * @return a writer instance
   */  
  shmdata_base_writer_t *shmdata_base_writer_init ();

  /** 
   * Initialization function that set the file path for the shared memory.
   * 
   * @param socketPath is the file name of the shared memory  
   *
   * @return TRUE if set, FALSE if the file already exists. 
   * If the file exists, this function shoud be called again.
   */  
  gboolean shmdata_base_writer_set_path (shmdata_base_writer_t * writer,
					 const char *socketPath);

  
  /** 
   * Initialization function that hot plug the shared memory writer to a GStreamer element.
   * 
   * @param pipeline is the pipeline where the base writer will be added
   * @param Element is the element which src pad will provide data to write
   * This element is assumed to be added in the pipeline
   * 
   */  
  void shmdata_base_writer_plug (shmdata_base_writer_t * writer,
				 GstElement * pipeline,
				 GstElement * Element);
  
  /** 
   * Alternative to shmdata_base_writer_plug,
   * using a pad instead of an element. 
   * 
   * @param pipeline is the pipeline where the base writer will be added
   * @param srcPad is the src pad of the element  
   * 
   */
  void shmdata_base_writer_plug_pad (shmdata_base_writer_t * writer,
				     GstElement *pipeline,
				     GstPad     *srcPad);

  /** 
   * Close the writer (freeing memory, removing internal GStreamer elements 
   * from the pipeline and closing the shared memory socket). 
   * 
   * @param writer is the base writer to close. 
   */
  void shmdata_base_writer_close (shmdata_base_writer_t * writer);

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHMDATA_BASE_WRITER_H_

/** @}*/
