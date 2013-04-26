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

#include "vorbis.h"
#include "gst-utils.h"

namespace switcher
{
  QuiddityDocumentation Vorbis::doc_ ("audio encoder", "vorbis",
				      "Vorbis encoder (up to 255 interleaved channels)");
  
  bool
  Vorbis::init ()
  {
    GstUtils::make_element ("bin",&vorbisbin_);
    GstUtils::make_element ("vorbisenc", &vorbisenc_);
    //g_object_set (G_OBJECT (bin_), "async-handling", TRUE, NULL);
    add_element_to_cleaner (vorbisenc_);
    add_element_to_cleaner (vorbisbin_);

    //set the name before registering properties
    set_name (gst_element_get_name (vorbisenc_));
    set_sink_element (vorbisbin_);
    set_on_first_data_hook (Vorbis::make_shmdata_writer,this);
    return true;
  }

  QuiddityDocumentation 
  Vorbis::get_documentation ()
  {
    return doc_;
  }
  
  void 
  Vorbis::make_shmdata_writer(ShmdataReader *caller, void *vorbisbin_instance)
  {
    Vorbis *context = static_cast<Vorbis *>(vorbisbin_instance);

    caller->set_sink_element (context->vorbisbin_);
    gst_bin_add (GST_BIN (context->bin_), context->vorbisbin_);

    //FIXME check for cleaning audioconvert
    GstElement *audioconvert;
    GstUtils::make_element ("audioconvert", &audioconvert);

    gst_bin_add_many (GST_BIN (context->vorbisbin_),
		      context->vorbisenc_,
		      audioconvert,
		      NULL);
    gst_element_link_many (audioconvert,
			   context->vorbisenc_,
			   NULL);
    
    GstUtils::sync_state_with_parent (context->vorbisbin_);

    GstPad *sink_pad = gst_element_get_static_pad (audioconvert, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (context->vorbisbin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
    
     GstCaps *vorbiscaps = gst_caps_new_simple ("audio/x-vorbis", NULL);
     ShmdataWriter::ptr vorbisframes_writer;
     vorbisframes_writer.reset (new ShmdataWriter ());
     std::string writer_name = context->make_file_name ("vorbisframes"); 
     vorbisframes_writer->set_path (writer_name.c_str());
     vorbisframes_writer->plug (context->bin_, context->vorbisenc_, vorbiscaps);
     context->register_shmdata_writer (vorbisframes_writer);
  }

}
