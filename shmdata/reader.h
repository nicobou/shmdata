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
#include <string>
#include <gst/gst.h>
#include <gio/gio.h>

namespace shmdata
{
   class Reader {
   public:
       Reader (const std::string socketPath, void(*on_first_video_data)( Reader *, void *), void *user_data);
       //Reader ();
       ~Reader ();
       //where to push the video data
       void setSink (GstElement *Pipeline, GstElement *sink); 
   private:
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
       std::string socketName_;
       //user callback
       void (*on_first_video_data_)(Reader *,void *);
       void* userData_;
       //state boolean
       gboolean initialized_; //the shared video has been attached once
       //dynamic linking
       void attach ();
       void detach ();
       //gcallbacks
       static gboolean clean_source (gpointer user_data);
       static GstBusSyncReply message_handler (GstBus *bus, GstMessage *msg, gpointer user_data);
       static void file_system_monitor_change (GFileMonitor *monitor, GFile *file, GFile *other_file, GFileMonitorEvent type, gpointer user_data);
       //log
       static void shmdata_log_handler (const gchar *log_domain, GLogLevelFlags log_level, const gchar *message, gpointer user_data);

   };

}      //end namespace  shmdata
#endif //_SHM_DATA_READER_H_

