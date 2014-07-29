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

#include "aravis-genicam.h"
#include "gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(AravisGenicam,
				       "GenICam Camera",
				       "genicam video", 
				       "Genicam video source using the Aravis library",
				       "LGPL",
				       "genicam",
				       "Nicolas Bouillot");
  AravisGenicam::AravisGenicam () :
    aravissrc_ (nullptr)
  {}

  bool
  AravisGenicam::init_gpipe ()
  {
    if (!GstUtils::make_element ("aravissrc", &aravissrc_))
      {
	g_debug ("aravissrc not available, install aravis (http://git.gnome.org/browse/aravis/)");
	return false;
      }
    set_name (gst_element_get_name (aravissrc_));
    
    //install_property (G_OBJECT (aravissrc_),"camera-name","aravissrc");

    install_property (G_OBJECT (aravissrc_),"gain","gain", "Gain");

    install_property (G_OBJECT (aravissrc_),"gain-auto","gain-auto", "Gain Auto");

    install_property (G_OBJECT (aravissrc_),"exposure","exposure", "Exposure");
    install_property (G_OBJECT (aravissrc_),"exposure-auto","exposure-auto", "Exposure Auto");
    install_property (G_OBJECT (aravissrc_),"h-binning","h-binning", "H-binning");
    install_property (G_OBJECT (aravissrc_),"v-binning","v-binning", "V-binning");
    install_property (G_OBJECT (aravissrc_),"offset-x","offset-x", "Offset-x");
    install_property (G_OBJECT (aravissrc_),"offset-y","offset-y", "Offset-y");


    install_method ("Capture",
		    "capture",
		    "start capturing from camera",
		    "success or fail",
		    Method::make_arg_description ("Name",
						  "name", 
						  "the genicam camera name obtained with the command arv-tool-0.2 or 'default')",
						  nullptr),
		    (Method::method_ptr)&start_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr), 
		    this);
    
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
      g_object_set (G_OBJECT (aravissrc_),"camera-name", name.c_str (), nullptr); 
    
    GstElement *colorspace;
    if (!GstUtils::make_element ("ffmpegcolorspace", &colorspace))
	return false;

    
    gst_bin_add_many (GST_BIN (bin_), aravissrc_, colorspace, nullptr);
    gst_element_link (aravissrc_, colorspace);
    //GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (aravissrc_);
    GstUtils::sync_state_with_parent (colorspace);
    

    GstPad *srcpad = gst_element_get_static_pad (colorspace, "src");
    //creating a shmdata
    ShmdataWriter::ptr connector;
    connector.reset (new ShmdataWriter ());
    std::string connector_name = make_file_name ("video");
    connector->set_path (connector_name.c_str());
    connector->plug (bin_, srcpad);
    register_shmdata (connector);

    g_message ("%s created a new shmdata writer (%s)", 
     	       get_nick_name ().c_str(), 
     	       connector_name.c_str ());

    return true;
  }

}
