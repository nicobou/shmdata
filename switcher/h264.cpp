/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/h264.h"

namespace switcher
{
  H264::H264 ()
  {
    h264bin_ = gst_element_factory_make ("bin",NULL);

    GstElement *h264 = gst_element_factory_make ("x264enc",NULL);
    GstElement *colorspace = gst_element_factory_make ("ffmpegcolorspace",NULL);

    gst_bin_add_many (GST_BIN (h264bin_),
		      h264,
		      colorspace,
		      NULL);
    gst_element_link (colorspace,h264);

    GstPad *sink_pad = gst_element_get_static_pad (colorspace, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (h264bin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);

    GstPad *src_pad = gst_element_get_static_pad (h264, "src");
    GstPad *ghost_srcpad = gst_ghost_pad_new (NULL, src_pad);
    gst_pad_set_active(ghost_srcpad,TRUE);
    gst_element_add_pad (h264bin_, ghost_srcpad); 
    gst_object_unref (src_pad);

    //set the name before registering properties
    name_ = gst_element_get_name (h264);

    set_sink_element (h264bin_);

    set_on_first_data_hook (H264::make_shmdata_writer,this);

  }

  BaseEntityDocumentation H264::doc_ ("video encoder", "x264enc",
				      "H264 encoder");

  BaseEntityDocumentation 
  H264::get_documentation ()
  {
    return doc_;
  }
  
  void 
  H264::make_shmdata_writer(void *h264bin_instance)
  {
    H264 *context = static_cast<H264 *>(h264bin_instance);
    
    GstCaps *h264caps = gst_caps_new_simple ("video/x-h264", NULL);
    ShmdataWriter::ptr h264frames_writer;
    h264frames_writer.reset (new ShmdataWriter ());
    std::string writer_name ("/tmp/switcher_pid_"+context->name_+"_h264frames"); 
    h264frames_writer->set_absolute_name (writer_name.c_str());
    h264frames_writer->plug (context->bin_, context->h264bin_, h264caps);
    context->shmdata_writers_.insert (writer_name, h264frames_writer);
  }

}
