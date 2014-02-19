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

/** \addtogroup libshmdata
 * provides hot plugging between GStreamer pipelines via a shared memory.
 * compile with `pkg-config --cflags --libs shmdata-0.6`
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
  typedef void (*shmdata_base_reader_on_first_data)(shmdata_base_reader_t *reader, 
						    void *user_data);

  /*! \fn void (*on_have_type)(shmdata_base_reader_t *reader, GstCaps *caps, void *userdata);
   *  \brief Callback triggered when a type for data passing through the reader has been found.  
   *  \param reader is the reader 
   *  \param caps is the caps of the data stream
   *  \param user_data is a pointer to the user data 
   */
  typedef void (*shmdata_base_reader_on_have_type)(shmdata_base_reader_t *reader, 
						   GstCaps *caps, 
						   void *user_data);

  /** 
   * \deprecated use shmdata_base_reader_new instead
   * Initialization function that starts monitoring the socket path. This install by default a sync_handler on 
   * the gst_bus. Using multiple readers will warn about trying to replace the sync handler, but does not 
   * affect the readers.   
   * 
   * @param socketPath is the file name of the shared memory
   * @param bin is the bin where the base writer will be added. This bin should already be added to a pipeline. 
   * @param on_first_data is the function pointer that will be called when 
   * the connecting with the writer. Your sink elements may need to be added to the bin and state synchronized in this function.   
   * @param user_data is the user data pointer for the on_first_video_data
   * callback
   * 
   * @return a base reader
   */
  shmdata_base_reader_t *shmdata_base_reader_init (const char *socketPath,
						   GstElement *bin,
						   shmdata_base_reader_on_first_data cb,
						   void *user_data);

  /** 
   * Initialization function that will replace shmdata_base_reader_init. 
   * Before starting the shmdata, one need to set the callback function and 
   * set if the Gstreamer bus sync handler (singleton) is managed by the library (see
   * shmdata_base_reader_install_sync_handler) 
   * 
   * @return a base reader
   */
  shmdata_base_reader_t *shmdata_base_reader_new ();



  /** 
   * Required configuration function set the on_first data callback. When called, the application 
   * must link with some sink element in order to get the stream. After being called, 
   * the function shmdata_base_reader_set_sink will have no effect on the reader 
   * 
   * 
   * @param cb is the function that will be called when 
   * the connecting with the writer. 
   * @param user_data is the user data for the callback
   * 
   */
  void shmdata_base_reader_set_callback (shmdata_base_reader_t *reader, 
					 shmdata_base_reader_on_first_data cb,
					 void *user_data);

  /** 
   * Set the user callback for being informed of the type found when data is starting 
   * to go through the reader.
   * 
   * 
   * @param cb is the function that will be called when 
   * the type has been found. 
   * @param user_data is the user data for the callback
   * 
   */
  void shmdata_base_reader_set_on_have_type_callback (shmdata_base_reader_t *reader,
						      shmdata_base_reader_on_have_type cb,
						      void *user_data);
  
  /** 
   * Configuration function that enable the choice of managing the gst_bus sync_handler by he library or
   * by the application
   *
   * @param install set the installation of the sync_handler by the library
   * 
   * The gst_bus sync_handler is singleton. In this case, using multiple shmdata readers with library manager
   * sync_handler will introduce warnings when starting the reader. The sync_handler installed by the library
   * is however able to work with multiple reader. In order to avoid the warning, the application can ensure
   * only one sync_handler is installed, using this function.
   * 
   * Alternatively, the application can install its own sync handler and have related error messages 
   * by the reader being processed. In this case, here is how to call error processing from 
   * your sync_handler:  
   @code
   //write the sync handler
    GstBusSyncReply my_handler (GstBus *bus,
                                GstMessage *msg, gpointer user_data) {
      shmdata_base_reader_t *reader = (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), 
                                                                                   "shmdata_base_reader");
      if ( reader != NULL)
        {
          if ( shmdata_base_reader_process_error (reader, msg)) 
     	    return GST_BUS_DROP; 
          else 
     	    return GST_BUS_PASS; 
        }
    
      return GST_BUS_PASS; 
    }

   //and install your sync_handler where desired
   gst_bus_set_sync_handler (bus, my_handler, NULL);  
   
   @endcode
   */
  void shmdata_base_reader_install_sync_handler (shmdata_base_reader_t *reader, 
						 gboolean install);


  /** 
   * Configuration function that set the bin in which the reader will be. If the sync_handler is used
   * by the reader, the bin must be in a top level pipeline before starting the reader.
   * 
   * @param bin is the bin where the base writer will be added. if This bin should already be added to a pipeline. 
   */
  gboolean shmdata_base_reader_set_bin (shmdata_base_reader_t *reader, 
					GstElement *bin);

  /** 
   * Configuration function that set the GMainContext in which the reader will be. 
   * 
   * @param context is the GMainContext. 
   */
  gboolean shmdata_base_reader_set_g_main_context (shmdata_base_reader_t *reader, 
						   GMainContext *context);
  
  /** 
   * Starting the reader. The reader will monitor the file socketPath. When created by a 
   * writer, the reader will open the socket and will forward the stream. 
   * 
   * @param socketPath is the file name of the shared memory
   */
  gboolean shmdata_base_reader_start (shmdata_base_reader_t *reader, 
				      const char *socketPath);


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
  void shmdata_base_reader_set_sink (shmdata_base_reader_t *reader,
				     GstElement *sink);


  /** 
   * Tell the reader to use an absolute timestamp, i.e. do not reset timestamp to 0
   * when connecting or reconnecting to the writer.
   * 
   * @param reader is the base reader to inform
   * @param do_absolute use absolute timestamp if != 0
   */
  void shmdata_base_reader_set_absolute_timestamp (shmdata_base_reader_t *reader,
                                                   gboolean do_absolute);



  /** 
   * Get a copy of the caps describing data passing through the shmdata reader.
   * 
   * @param reader is the base reader 
   *
   * @return the detected capabilities in stream or NULL
   */
  
  GstCaps *shmdata_base_reader_get_caps (shmdata_base_reader_t *reader);

  /**
   * Function to call in the gst_bus sync handler when it is installed by the application. 
   * See shmdata_base_reader_install_sync_handler for more details. 
   */ 

  gboolean shmdata_base_reader_process_error (shmdata_base_reader_t *reader, 
					      GstMessage *msg);


  /** 
   * Close the reader (freeing memory and removing internal GStreamer elements 
   * from the pipeline). 
   * 
   * @param reader is the base reader to close
   */
  void shmdata_base_reader_close (shmdata_base_reader_t *reader);

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHMDATA_BASE_READER_H_

/** @}*/
