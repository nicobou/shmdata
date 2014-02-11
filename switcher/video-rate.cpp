/*
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

#include "video-rate.h"
#include "gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(VideoRate,
				       "Video Rate",
				       "video converter", 
				       "Adjusts video frame rate (video/x-raw-yuv)",
				       "LGPL",
				       "videorate",
				       "Nicolas Bouillot");
  VideoRate::VideoRate () :
    video_rate_bin_ (NULL),
    video_rate_enc_ (NULL)
  {}

  bool
  VideoRate::init_segment ()
  {
    if (!GstUtils::make_element ("bin",&video_rate_bin_)
     	|| !GstUtils::make_element ("videorate",&video_rate_enc_))
      return false;
    
    //    g_object_set (G_OBJECT (bin_), "async-handling", TRUE, NULL);
    add_element_to_cleaner (video_rate_enc_);
    add_element_to_cleaner (video_rate_bin_);

    set_sink_element (video_rate_bin_);
    set_on_first_data_hook (VideoRate::make_shmdata_writer,this);
    return true;
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
