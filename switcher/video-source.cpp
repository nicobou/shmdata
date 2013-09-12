/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "video-source.h"
#include "gst-utils.h"

namespace switcher
{
  VideoSource::VideoSource () 
  {
    video_tee_ = NULL;
    videocaps_ = gst_caps_new_simple ("video/x-raw-yuv",
				      // "format", GST_TYPE_FOURCC,
				      // GST_MAKE_FOURCC ('U', 'Y', 'V', 'Y'),
				      // "format", GST_TYPE_FOURCC,
				      // GST_MAKE_FOURCC ('I', '4', '2', '0'),
				      //"framerate", GST_TYPE_FRACTION, 30, 1,
				      // "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, 
				      //  "width", G_TYPE_INT, 640, 
				      //  "height", G_TYPE_INT, 480,
				      NULL);
    
    init_startable (this);
  }

  VideoSource::~VideoSource () 
  {
    gst_caps_unref (videocaps_);
  }

  void 
  VideoSource::make_tee () 
  {
    if (!GstUtils::make_element ("tee",&video_tee_))
      g_warning ("VideoSource: tee element is mandatory\n");
    gst_bin_add_many (GST_BIN (bin_),
      		      video_tee_,
      		      NULL);
  }
  
 
  bool 
  VideoSource::start ()
  {
    rawvideo_ = NULL;
    if (!make_video_source (&rawvideo_))
      {
	g_debug ("cannot make video source");
	return false;
      }
    
    unregister_shmdata_writer (shmdata_path_);
    reset_bin ();
    make_tee ();
    
    gst_bin_add (GST_BIN (bin_), rawvideo_);
    gst_element_link (rawvideo_, video_tee_);
    
    ShmdataWriter::ptr shmdata_writer;
    shmdata_writer.reset (new ShmdataWriter ());
    shmdata_path_ = make_file_name ("video"); 
    shmdata_writer->set_path (shmdata_path_.c_str());
    shmdata_writer->plug (bin_, video_tee_, videocaps_);
    register_shmdata_writer (shmdata_writer);
    GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (rawvideo_);
    GstUtils::sync_state_with_parent (video_tee_);
 
    return on_start ();
  }
  
  bool 
  VideoSource::stop ()
  {
    bool res = on_stop ();
    unregister_shmdata_writer (shmdata_path_);
    reset_bin ();
    return res;
  }
  

}
