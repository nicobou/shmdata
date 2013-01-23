/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#ifndef _SHMDATA_ANY_WRITER_H_
#define _SHMDATA_ANY_WRITER_H_

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

  /** \addtogroup libshmdata-any
   *  @{
   */
  
  /**
   * @file   any-data-writer.h
   * 
   * @brief  Writing any kind of data flow to a shared memory 
   * 
   * The any writer provides a shared memory writer, allowing transmission 
   * of any kind data flows to a shared memory. It supports hot connections 
   * and disconnections of multiple reader. A single writer is allowed for 
   * a given shared memory (socket path).
   *
   * Data are pushed to the writer, with a mandatory timestamp. 
   */


  typedef struct shmdata_any_writer_ shmdata_any_writer_t;

  /** 
   * Initialization function.
   * 
   * @return a writer instance 
   */
  shmdata_any_writer_t *shmdata_any_writer_init ();

  /** 
   * Initialization function that set the file path for the shared memory.
   * 
   * @param socketPath is the file name of the shared memory  
   *
   * @return 1 if set, 0 if the file already exists. 
   * If the file exists, the path is not set.
   */  
  int shmdata_any_writer_set_path (shmdata_any_writer_t * writer,
				   const char *socketPath);


  /** 
   * Set function for writer debugging. Debugging is disabled by default.
   * 
   * @param writer is the writer that needs to be set
   * @param debug is either SHMDATA_ENABLE_DEBUG or SHMDATA_DISABLE_DEBUG  
   */
  void shmdata_any_writer_set_debug (shmdata_any_writer_t *writer,
				     int debug);

  /** 
   * Set function for describing the type of data to be transmitted. 
   * This function should be called before shmdata_any_writer_start.
   * 
   * @param writer is the writer to update 
   * @param type is a string describing the data. It can also be a string 
   * describing a GStreamer caps as provided by the gst_caps_to_string 
   * function, enabling compatibility with a base reader.  
   */
  void shmdata_any_writer_set_data_type (shmdata_any_writer_t *writer,
					 const char *type);

  /** 
   * Start to write.
   * 
   * @param writer is the writer to start
   */
  void shmdata_any_writer_start (shmdata_any_writer_t *writer);


  typedef struct shmdata_base_reader_ shmdata_base_reader_t; 

  /*! \fn void (*shmdata_any_writer_done_with_data)(void *user_data);
   *  \brief triggered when the writer is done with the your data. 
   *  \param user_data is a pointer to your data. 
   */  
  typedef void (*shmdata_any_writer_done_with_data)(void *user_data);

  /** 
   * Writing to the shared memory. The done_with_data callback is provided in 
   * order to inform when data are not used anymore by the writer, and therefore
   * can be freed if necessary. 
   * 
   * @param writer is the writer that should push the data 
   * @param data is a pointer to the data to transmit
   * @param size is the size of the data to transmit
   * @param timestamp is the timestamp associated to the data
   * @param done_with_data is the callback informing when data can be freed
   * @param user_data is the user data for the done_with_data callback
   */
  void shmdata_any_writer_push_data (shmdata_any_writer_t *writer,
				     void *data,
				     int size,
				     unsigned long long timestamp,
				     shmdata_any_writer_done_with_data cb,
				     void *user_data);

  /** 
   * same as shmdata_any_writer_push_data, but adds buffer duration, as required for some 
   * data (audio for instance).
   * 
   * @param writer is the writer that should push the data 
   * @param data is a pointer to the data to transmit
   * @param size is the size of the data to transmit
   * @param timestamp is the timestamp associated to the data
   * @param duration is the duration of data contained in the buffer
   * @param done_with_data is the callback informing when data can be freed
   * @param user_data is the user data for the done_with_data callback
   */
  void shmdata_any_writer_push_data_with_duration (shmdata_any_writer_t *writer,
						   void *data,
						   int size,
						   unsigned long long timestamp,
						   unsigned long long duration,
						   unsigned long long offset,
						   unsigned long long offeset_end,
						   shmdata_any_writer_done_with_data cb,
						   void *user_data);

  /** 
   * Close the writer (free memory and close shared memory socket). 
   * 
   * @param writer is the writer to close
   */
  void shmdata_any_writer_close (shmdata_any_writer_t *writer);

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHMDATA_ANY_WRITER_H_


/** @}*/
