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

#ifndef _SHM_DATA_READER_H_
#define _SHM_DATA_READER_H_

#include <gst/gst.h>
#include <gio/gio.h>

typedef struct shmdata_reader_ shmdata_reader_t;
struct shmdata_reader_ {
    //pipeline elements
    GstElement *pipeline_;
    GstElement *source_;
    GstElement *deserializer_;
    GstElement *sink_;
    GstPad *sinkPad_;
    GstPad *deserialPad_;
    //monitoring the shm file
    GFile *shmfile_; 
    GFileMonitor* dirMonitor_;
    const char *socketName_;
    //user callback
    void (*on_first_video_data_)(shmdata_reader_t *,void *);
    void* userData_;
    //state boolean
    gboolean initialized_; //the shared video has been attached once
};

shmdata_reader_t *shmdata_reader_init (const char *socketPath, 
				       void(*on_first_video_data)(shmdata_reader_t *, void *), 
				       void *user_data);

gboolean shmdata_reader_close(shmdata_reader_t *reader);

//where to push the video data
void shmdata_reader_set_sink (shmdata_reader_t *reader,
			      GstElement *Pipeline, 
			      GstElement *sink); 

#endif //_SHM_DATA_READER_H_

