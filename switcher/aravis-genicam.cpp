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

#include "aravis-genicam.h"
#include "gst-utils.h"

namespace switcher
{

  QuiddityDocumentation AravisGenicam::doc_ ("video source", "genicam",
						   "Genicam video source using the Aravis library");
  
  
  bool
  AravisGenicam::init ()
  {
    if (!GstUtils::make_element ("aravissrc", &aravissrc_))
      {
	g_warning ("aravissrc not available, install aravis (http://git.gnome.org/browse/aravis/)");
	return false;
      }
    set_name (gst_element_get_name (aravissrc_));
    
    //register_property (G_OBJECT (aravissrc_),"camera-name","aravissrc");

    register_property (G_OBJECT (aravissrc_),"gain","gain");

    register_property (G_OBJECT (aravissrc_),"gain-auto","gain-auto");

    register_property (G_OBJECT (aravissrc_),"exposure","exposure");
    register_property (G_OBJECT (aravissrc_),"exposure-auto","exposure-auto");
    register_property (G_OBJECT (aravissrc_),"h-binning","h-binning");
    register_property (G_OBJECT (aravissrc_),"v-binning","v-binning");
    register_property (G_OBJECT (aravissrc_),"offset-x","offset-x");
    register_property (G_OBJECT (aravissrc_),"offset-y","offset-y");


    //registering add_data_stream
    register_method("start",
		    (void *)&start_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("start", 
			    "start the stream from camera", 
			    Method::make_arg_description ("name", 
							  "the genicam camera name obtained with the command arv-tool-0.2 or 'default')",
							  NULL));
    return true;
  }
  
  gboolean
  AravisGenicam::start_wrapped (gpointer name, 
				gpointer user_data)
  {
    AravisGenicam *context = static_cast<AravisGenicam *>(user_data);
  
    if (context->start ((char *)name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  AravisGenicam::start (std::string name)
  {
    g_debug ("Genicam using camera %s", name.c_str ());
    if (g_strcmp0 (name.c_str (), "default") != 0 && g_strcmp0 (name.c_str (), "Default") != 0)
      g_object_set (G_OBJECT (aravissrc_),"camera-name", name.c_str (), NULL); 
    
    GstElement *colorspace;
    if (!GstUtils::make_element ("ffmpegcolorspace", &colorspace))
	return false;

    
    gst_bin_add_many (GST_BIN (bin_), aravissrc_, colorspace, NULL);
    gst_element_link (aravissrc_, colorspace);
    GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (aravissrc_);
    GstUtils::sync_state_with_parent (colorspace);
    

    GstPad *srcpad = gst_element_get_static_pad (colorspace, "src");
    //creating a shmdata
    ShmdataWriter::ptr connector;
    connector.reset (new ShmdataWriter ());
    std::string connector_name = make_file_name ("video");
    connector->set_path (connector_name.c_str());
    connector->plug (bin_, srcpad);
    register_shmdata_writer (connector);

    g_message ("%s created a new shmdata writer (%s)", 
     	       get_nick_name ().c_str(), 
     	       connector_name.c_str ());

    return true;
  }
  
  
  QuiddityDocumentation 
  AravisGenicam::get_documentation ()
  {
    return doc_;
  }
  
}
