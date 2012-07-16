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

#ifndef _SHMDATA_OSG_READER_IMPL_H_
#define _SHMDATA_OSG_READER_IMPL_H_

#include <osg/Texture2D>
#include <osg/Image>
#include <gst/gst.h>
#include "shmdata/base-reader.h"

namespace shmdata
{
  class OsgReader_impl
  {
  public:
    OsgReader_impl ();
    bool setPath (const std::string &socketPath);
    void play ();
    void pause ();
    int getWidth ();
    int getHeight();
    osg::Texture2D * getTexture ();
     ~OsgReader_impl ();
    void setDebug (bool debug);
  private:
    const std::string * socketName_;
    shmdata_base_reader_t *reader_;
    GstBuffer *last_buffer_;
    osg::Texture2D *texture_;
    osg::Image *image_;
    GstElement *pipeline_;
    GThread *sharedVideoThread_;
    GMainLoop *loop_;
    bool debug_;
    bool playing_;
    int width_;
    int height_;
    static void log_handler (const gchar * log_domain,
			     GLogLevelFlags log_level,
			     const gchar * message, gpointer user_data);
    static void on_new_buffer_from_source (GstElement * elt,
					   gpointer user_data);
    static void on_first_video_data (shmdata_base_reader_t * reader,
				     void *user_data);
    static void g_loop_thread (gpointer user_data);
  };

} //end namespace

#endif //_SHMDATA_READER_IMPL_H_

