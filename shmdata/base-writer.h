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

#ifndef _SHMDATA_BASE_WRITER_H_
#define _SHMDATA_BASE__WRITER_H_

#include <gst/gst.h>
#include <gio/gio.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct shmdata_base_writer_ shmdata_base_writer_t;

  shmdata_base_writer_t *shmdata_base_writer_init (const char *socketPath,
						   GstElement * pipeline,
						   GstElement * Element);

  shmdata_base_writer_t *shmdata_base_writer_init_pad (const char *socketPath,
						       GstElement * pipeline,
						       GstPad * srcPad);

  void shmdata_base_writer_close (shmdata_base_writer_t * writer);

#ifdef __cplusplus
}
#endif				/* extern "C" */
#endif				//_SHMDATA_BASE_WRITER_H_
