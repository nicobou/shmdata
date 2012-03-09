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

#ifndef _SHM_DATA_WRITER_H_
#define _SHM_DATA_WRITER_H_

#include <gst/gst.h>

typedef struct shmdata_writer_ shmdata_writer_t;
struct shmdata_writer_ {
    GstElement *qserial_;
    GstElement *serializer_;
    GstElement *shmsink_;
    GstElement *pipeline_;
    gboolean timereset_;
    GstClockTime timeshift_;
};

shmdata_writer_t *shmdata_writer_init (const char *socketPath,
				       GstElement *pipeline,
				       GstElement *Element);

shmdata_writer_t *shmdata_writer_init_pad (const char *socketPath,
					   GstElement *pipeline,
					   GstPad *srcPad);

gboolean shmdata_writer_close (shmdata_writer_t *writer);

#endif //_SHM_DATA_WRITER_H_

