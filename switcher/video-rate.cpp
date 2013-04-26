/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "video-rate.h"
#include "gst-utils.h"

namespace switcher
{
  QuiddityDocumentation VideoRate::doc_ ("rate", "videorate",
					 "Adjusts video frame rate (video/x-raw-yuv)");
  
  bool
  VideoRate::init ()
  {
    if (!GstUtils::make_element ("bin",&video_rate_bin_)
	|| !GstUtils::make_element ("videorate",&video_rate_enc_))
      return false;
    
    //    g_object_set (G_OBJECT (bin_), "async-handling", TRUE, NULL);
    add_element_to_cleaner (video_rate_enc_);
    add_element_to_cleaner (video_rate_bin_);

    //set the name before registering properties
    set_name (gst_element_get_name (video_rate_enc_));
    set_sink_element (video_rate_bin_);
    set_on_first_data_hook (VideoRate::make_shmdata_writer,this);
    return true;
  }

  QuiddityDocumentation 
  VideoRate::get_documentation ()
  {
    return doc_;
  }
  
  void 
  VideoRate::make_shmdata_writer(ShmdataReader *caller, void *video_rate_bin_instance)
  {
    VideoRate *context = static_cast<VideoRate *>(video_rate_bin_instance);

    caller->set_sink_element (context->video_rate_bin_);
    gst_bin_add (GST_BIN (context->bin_), context->video_rate_bin_);

    GstCaps *video_rate_caps = gst_caps_from_string ("video/x-raw-yuv, framerate=(fraction)30/1");
    //FIXME clean that 
    GstElement *capsfilter;
    if (!GstUtils::make_element ("capsfilter",&capsfilter))
      return;

    g_object_set (G_OBJECT (capsfilter), "caps", video_rate_caps,NULL);

    gst_bin_add_many (GST_BIN (context->video_rate_bin_),
		      context->video_rate_enc_,
		      capsfilter,
		      NULL);

    gst_element_link (context->video_rate_enc_, capsfilter);

    GstUtils::sync_state_with_parent (context->video_rate_bin_);

    GstPad *sink_pad = gst_element_get_static_pad (context->video_rate_enc_, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (context->video_rate_bin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
    
    ShmdataWriter::ptr video_rate_frames_writer;
    video_rate_frames_writer.reset (new ShmdataWriter ());
    std::string writer_name = context->make_file_name ("video"); 
    video_rate_frames_writer->set_path (writer_name.c_str());
    video_rate_frames_writer->plug (context->bin_, capsfilter, video_rate_caps);
    context->register_shmdata_writer (video_rate_frames_writer);
  }

}
