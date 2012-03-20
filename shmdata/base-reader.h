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

#ifndef _SHMDATA_BASE_READER_H_
#define _SHMDATA_BASE_READER_H_

#include <gst/gst.h>
#include <gio/gio.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct shmdata_base_reader_ shmdata_base_reader_t;

  shmdata_base_reader_t *shmdata_base_reader_init (const char *socketPath,
						   GstElement * Pipeline,
						   void (*on_first_data)
						   (shmdata_base_reader_t *,
						    void *), void *user_data);

//where to push the video data
  void shmdata_base_reader_set_sink (shmdata_base_reader_t * reader,
//                            GstElement *Pipeline,
				     GstElement * sink);

  void shmdata_base_reader_close (shmdata_base_reader_t * reader);

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHM_DATA_READER_H_
