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

#ifndef _SHMDATA_ANY_READER_H_
#define _SHMDATA_ANY_READER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef SHMDATA_ENABLE_DEBUG
#define SHMDATA_ENABLE_DEBUG 1
#endif
#ifndef SHMDATA_DISABLE_DEBUG
#define SHMDATA_DISABLE_DEBUG 0
#endif

  /**
   * @file   any-data-reader.h
   * 
   * @brief  Reading any kind of data flow to a shared memory 
   * 
   * 
   */

  typedef struct shmdata_any_reader_ shmdata_any_reader_t;

  /** 
   * Initialization function.
   * 
   * @return a reader instance 
   */
  shmdata_any_reader_t *shmdata_any_reader_init ();

  /** 
   * Set function for reader debuging. Debuging is disabled by default.
   * 
   * @param reader is the reader that needs to be set
   * @param debug is either SHMDATA_ENABLE_DEBUG or SHMDATA_DISABLE_DEBUG  
   */
  void shmdata_any_reader_set_debug (shmdata_any_reader_t *reader,
				     int debug);

  /** 
   * Set function for registering data reception callback.
   * 
   * @param reader is the reader to set
   * @param on_data is the callback allowing receiving data 
   * @param user_data is the user data pointer for the on_first_video_data
   * callback
   */
  void shmdata_any_reader_set_on_data_handler (shmdata_any_reader_t *reader,
					       void (*on_data)
					       (shmdata_any_reader_t *,
						void *, void *, int,
						unsigned long long,
						const char *, void *),
					       void *user_data);


  void shmdata_any_reader_free (void *shmbuf);

  /** 
   * Set funtion for describing the type of data to be received. Any data not 
   * maching this descrption are not received. If not called, no data are filtered.
   * 
   * @param reader is the reader to update 
   * @param type is a string describing the data. It can also be a string 
   * describing a GStreamer caps as provided by the gst_caps_to_string 
   * function, enabling compatibility with a base writer.  
   */
  void shmdata_any_reader_set_data_type (shmdata_any_reader_t *reader,
					 const char *type);

  /** 
   * Start reading from the shared memory.
   * 
   * @param reader is the reader to start
   * @param socketPath is the file name of the shared memory
   */
  void shmdata_any_reader_start (shmdata_any_reader_t *reader,
				 const char *socketPath);

  /** 
   * Close the reader. 
   * 
   * @param reader is the reader to close
   */
  void shmdata_any_reader_close (shmdata_any_reader_t *reader);

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHMDATA_ANY_READER_H_

