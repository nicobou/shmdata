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


#ifndef _SHMDATA_ANY_READER_H_
#define _SHMDATA_ANY_READER_H_

#ifdef __cplusplus
extern "C"
{
#endif

  //FIXME remove that 
#ifndef SHMDATA_ENABLE_DEBUG
#define SHMDATA_ENABLE_DEBUG 1
#endif
#ifndef SHMDATA_DISABLE_DEBUG
#define SHMDATA_DISABLE_DEBUG 0
#endif

#ifndef SHMDATA_TRUE
#define SHMDATA_TRUE 1
#endif
#ifndef SHMDATA_FALSE
#define SHMDATA_FALSE 0
#endif

#ifndef SHMDATA_ENABLE_ABSOLUTE_TIMESTAMP
#define SHMDATA_ENABLE_ABSOLUTE_TIMESTAMP 1
#endif
#ifndef SHMDATA_DISABLE_ABSOLUTE_TIMESTAMP
#define SHMDATA_DISABLE_ABSOLUTE_TIMESTAMP 0
#endif

  /** \addtogroup libshmdata-any
   * provides sharing of custom data flows between processes.
   * compile with `pkg-config --cflags --libs shmdata-any-0.4`
   *  @{
   */

  /**
   * @file   any-data-reader.h
   * 
   * @brief  Reading any kind of data flow to a shared memory 
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
   * Set function for reader debugging. Debugging is disabled by default.
   * 
   * @param reader is the reader that needs to be set
   * @param debug is either SHMDATA_TRUE or SHMDATA_FALSE  
   */
  void shmdata_any_reader_set_debug (shmdata_any_reader_t *reader,
				     int debug);

  /** 
   * Set function running or not the gmainloop from the shmdata. The shmdata -needs- a gmainloop, 
   * accordingly set if to SHMDATA_FALSE only if you are running a gmainloop. Default behaviour 
   * is to run the gmainloop internally. 
   * 
   * @param reader is the reader that needs to be set
   * @param run is either SHMDATA_TRUE or SHMDATA_FALSE  
   */
  void shmdata_any_reader_run_gmainloop (shmdata_any_reader_t *reader,
					 int run);


  /*! \fn void (*shmdata_any_reader_on_data)(shmdata_any_reader_t *,void *, void *, int, unsigned long long, const char *, void *);
   *  \brief Callback triggered when a data frame has been written to the shared memory. You must free shmbuf when done using the shmdata_any_reader_free function.
   *  \param shmbuf is the pointer used to free the buffer with shmdata_any_reader_free when the reader is done with the data
   *  \param data is the data written by the writer
   *  \param data_size is the size of data
   *  \param timestamp is the date the writer has associated to the data
   *  \param type_description is a string describing the data type
   *  \param user_data is the user data
   */
  typedef void (*shmdata_any_reader_on_data)(shmdata_any_reader_t * reader,
					     void *shmbuf,
					     void *data,
					     int data_size,
					     unsigned long long timestamp,
					     const char *type_description, 
					     void *user_data);
  
  /** 
   * Free the received buffer.
   * 
   * @param shmbuf is the buffer obtained with the shmdata_any_reader_on_data callback
   */
  void shmdata_any_reader_free (void *shmbuf);



  /** 
   * Set function for registering data reception callback.
   * 
   * @param reader is the reader to set
   * @param on_data is the callback allowing receiving data 
   * @param user_data is the user data pointer for the on_first_video_data
   * callback
   */
  void shmdata_any_reader_set_on_data_handler (shmdata_any_reader_t *reader,
					       shmdata_any_reader_on_data cb,    
					       void *user_data);



  /** 
   * Set function for describing the type of data to be received. Any data not 
   * matching this description are not received. If not called, no data are filtered.
   * 
   * @param reader is the reader to update 
   * @param type is a string describing the data. It can also be a string 
   * describing a GStreamer caps as provided by the gst_caps_to_string 
   * function, enabling compatibility with a base writer.  
   */
  void shmdata_any_reader_set_data_type (shmdata_any_reader_t *reader,
					 const char *type);

  /** 
   * Tell the reader to use an absolute timestamp, i.e. do not reset timestamp to 0
   * when connecting or reconnecting to the writer.
   * 
   * @param reader is the any reader to inform
   * @param do_absolute use absolute timestamp if set to SHMDATA_ENABLE_ABSOLUTE_TIMESTAMP
   */
  void shmdata_any_reader_set_absolute_timestamp (shmdata_any_reader_t * reader,
                                                  int do_absolute);

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

  /** 
   * Clean up any resources created 
   *
   * This is actually wrapping gst_deinit(). It is normally not needed to call this 
   * function in a normal application as the resources will automatically be freed 
   * when the program terminates. This function is therefore mostly used by testsuites
   * and other memory profiling tools. After this call GStreamer (including this method) 
   * should not be used anymore. Close the reader. 
   */

  void shmdata_any_reader_clean_before_exiting ();

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHMDATA_ANY_READER_H_

/** @}*/
