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
#include <string>
#include <gst/gst.h>

namespace shmdata
{
    class Writer {
    public:
	Writer (const std::string socketPath,GstElement *pipeline,GstElement *Element);
	Writer (const std::string socketPath,GstElement *pipeline,GstPad *srcPad);
	~Writer ();
    private:
	GstElement *qserial_;
	GstElement *serializer_;
	GstElement *shmsink_;
	GstElement *pipeline_;
	gboolean timereset_;
	GstClockTime timeshift_;
	void make_shm_branch(const std::string socketPath);
	void set_branch_state_as_pipeline ();
	void link_branch(GstElement *srcElement);
	void link_branch(GstPad *srcPad);
	static gboolean reset_time (GstPad * pad, GstMiniObject * mini_obj, gpointer user_data);
	static void pad_unblocked (GstPad * pad, gboolean blocked, gpointer user_data);
	static void switch_to_new_serializer (GstPad * pad, gboolean blocked, gpointer user_data );
	static void on_client_connected (GstElement * shmsink, gint num, gpointer user_data); 
	static void shmdata_log_handler (const gchar *log_domain, GLogLevelFlags log_level, const gchar *message, gpointer user_data);

    };


}      //end namespace  shmdata
#endif //_SHM_DATA_WRITER_H_

