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

#include "jpegenc.h"
#include "switcher/gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JpegEnc,
				       "JPEG Encoder",
				       "video encoder", 
				       "JPEG encoder",
				       "LGPL",
				       "jpegenc",
				       "Nicolas Bouillot");
  JpegEnc::JpegEnc () :
    jpegencbin_ (NULL),
    jpegencenc_ (NULL)
  {}
 
  bool
  JpegEnc::init_segment ()
  {
    if (!GstUtils::make_element ("bin", &jpegencbin_)
	|| !GstUtils::make_element ("jpegenc", &jpegencenc_))
      return false;

    add_element_to_cleaner (jpegencenc_);
    add_element_to_cleaner (jpegencbin_);

    install_property (G_OBJECT (jpegencenc_),"quality","quality", "Quality");
    install_property (G_OBJECT (jpegencenc_),"idct-method","idct-method", "IDCT Algorithm");

    set_on_first_data_hook (JpegEnc::make_shmdata_writer,this);
    return true;
  }
  
  void 
  JpegEnc::make_shmdata_writer(ShmdataReader *caller, void *jpegencbin_instance)
  {
    JpegEnc *context = static_cast<JpegEnc *>(jpegencbin_instance);

    caller->set_sink_element (context->jpegencbin_);
    gst_bin_add (GST_BIN (context->bin_), context->jpegencbin_);

    GstElement *colorspace;
    GstUtils::make_element ("ffmpegcolorspace", &colorspace);

    gst_bin_add_many (GST_BIN (context->jpegencbin_),
		      context->jpegencenc_,
		      colorspace,
		      NULL);
    gst_element_link_many (colorspace,
			   context->jpegencenc_,
			   NULL);
    
    GstUtils::sync_state_with_parent (context->jpegencbin_);

    GstPad *sink_pad = gst_element_get_static_pad (colorspace, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (context->jpegencbin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
    
    GstCaps *jpegenccaps = gst_caps_new_simple ("image/jpeg", NULL);
    ShmdataWriter::ptr jpegencframes_writer;
    jpegencframes_writer.reset (new ShmdataWriter ());
    std::string writer_name = context->make_file_name ("jpegframes"); 
    jpegencframes_writer->set_path (writer_name.c_str());
    jpegencframes_writer->plug (context->bin_, context->jpegencenc_, jpegenccaps);
    context->register_shmdata_writer (jpegencframes_writer);
  }

}
