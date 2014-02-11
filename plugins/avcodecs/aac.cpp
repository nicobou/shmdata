/*
 * This file is part of switcher-avcodecs.
 *
 * switcher-avcodecs is free software: you can redistribute it and/or modify
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

#include "aac.h"
#include "switcher/gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(AAC, 
				       "AAC encoder",
				       "audio encoder", 
				       "AAC encoder (2 channels max)",
      				       "GPL",
				       "aacenc",
				       "Nicolas Bouillot");
  AAC::AAC () :
    aacbin_ (NULL),
    aacenc_ (NULL)
  {}

  bool
  AAC::init_segment ()
  {
    GstUtils::make_element ("bin", &aacbin_);
    //g_object_set (G_OBJECT (bin_), "async-handling", TRUE, NULL);
    
    if (GstUtils::make_element ("ffenc_aac", &aacenc_))
      {
	add_element_to_cleaner (aacenc_);
	//set the name before registering properties
	set_name (gst_element_get_name (aacenc_));
	install_property (G_OBJECT (aacenc_),"bitrate","bitrate", "Bitrate (bps)");
	set_on_first_data_hook (AAC::make_shmdata_writer,this);
	add_element_to_cleaner (aacbin_);
	set_sink_element (aacbin_);
	return true;
      }
    else
      {
	set_name (gst_element_get_name (aacbin_));
	set_sink_element (aacbin_);
	add_element_to_cleaner (aacbin_);
	return false;
      }
  }

  
  void 
  AAC::make_shmdata_writer(ShmdataReader *caller, void *aacbin_instance)
  {
    AAC *context = static_cast<AAC *>(aacbin_instance);

    caller->set_sink_element (context->aacbin_);
    gst_bin_add (GST_BIN (context->bin_), context->aacbin_);

    //FIXME check for cleaning audioconvert
    GstElement *audioconvert;
    if (!GstUtils::make_element ("audioconvert",&audioconvert))
      {
	g_warning ("audioconvert element is required for having aac encorder to work\n");
	return;
      }
      
    gst_bin_add_many (GST_BIN (context->aacbin_),
		      context->aacenc_,
		      audioconvert,
		      NULL);
    gst_element_link_many (audioconvert,
			   context->aacenc_,
			   NULL);
    
    GstUtils::sync_state_with_parent (context->aacbin_);

    GstPad *sink_pad = gst_element_get_static_pad (audioconvert, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (context->aacbin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
    
     GstCaps *aaccaps = gst_caps_new_simple ("audio/mpeg", NULL);
     ShmdataWriter::ptr aacframes_writer;
     aacframes_writer.reset (new ShmdataWriter ());
     std::string writer_name = context->make_file_name ("aacframes"); 
     aacframes_writer->set_path (writer_name.c_str());
     aacframes_writer->plug (context->bin_, context->aacenc_, aaccaps);
     context->register_shmdata_writer (aacframes_writer);
    
  }

}
