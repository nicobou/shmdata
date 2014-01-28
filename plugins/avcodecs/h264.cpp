/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-avcodecs.
 *
 * switcher  avcodecs is free software: you can redistribute it and/or modify
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

#include "h264.h"
#include "switcher/gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(H264,
				       "H264 Encoder",
				       "video encoder", 
				       "H264 encoder",
				       "GPL",
				       "x264enc",
				       "Nicolas Bouillot");
  H264::H264 () :
    h264bin_ (NULL),
    h264enc_ (NULL)
  {}

  bool
  H264::init_segment ()
  {
    if (!GstUtils::make_element ("bin", &h264bin_)
	|| !GstUtils::make_element ("x264enc", &h264enc_))
      return false;

    g_object_set (G_OBJECT (h264enc_), 
		  "speed-preset",1, 
		  "bitrate",10000,
                  "threads", 4,
		  //"sliced-threads", TRUE,
		  NULL);
    add_element_to_cleaner (h264enc_);
    add_element_to_cleaner (h264bin_);

    install_property (G_OBJECT (h264enc_),"speed-preset","speed-preset", "Speed Preset");
    install_property (G_OBJECT (h264enc_),"interlaced","interlaced", "Optimize For Interlaced Video");
    install_property (G_OBJECT (h264enc_),"bitrate","bitrate", "Bitrate (kbps)");

    set_on_first_data_hook (H264::make_shmdata_writer,this);
    return true;
  }
  
  void 
  H264::make_shmdata_writer(ShmdataReader *caller, void *h264bin_instance)
  {
    H264 *context = static_cast<H264 *>(h264bin_instance);

    caller->set_sink_element (context->h264bin_);
    gst_bin_add (GST_BIN (context->bin_), context->h264bin_);

    GstElement *colorspace;
    GstUtils::make_element ("ffmpegcolorspace", &colorspace);

    gst_bin_add_many (GST_BIN (context->h264bin_),
		      context->h264enc_,
		      colorspace,
		      NULL);
    gst_element_link_many (colorspace,
			   context->h264enc_,
			   NULL);
    
    GstUtils::sync_state_with_parent (context->h264bin_);

    GstPad *sink_pad = gst_element_get_static_pad (colorspace, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (context->h264bin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
    
     GstCaps *h264caps = gst_caps_new_simple ("video/x-h264", NULL);
     ShmdataWriter::ptr h264frames_writer;
     h264frames_writer.reset (new ShmdataWriter ());
     std::string writer_name = context->make_file_name ("h264frames"); 
     h264frames_writer->set_path (writer_name.c_str());
     h264frames_writer->plug (context->bin_, context->h264enc_, h264caps);
     context->register_shmdata_writer (h264frames_writer);
  }

}
