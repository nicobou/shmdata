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

#include "shmdata-from-gdp-file.h"
#include "gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataFromGDPFile,
				       "Shmdata File Player",
				       "file source", 
				       "play file(s) recorded with shmdatatofile",
				       "LGPL",
				       "shmfromfile",
				       "Nicolas Bouillot");
  
  ShmdataFromGDPFile::~ShmdataFromGDPFile ()
  {
    clean_players ();
  }

  bool 
  ShmdataFromGDPFile::init ()
  {

    publish_method ("Add File",
		    "add_file", 
		    "add a file to play", 
		    "success or fail",
		    Method::make_arg_description ("File Path",
						  "filepath", 
						  "file location",
						  "Shmdata Path",
						  "shmpath",
						  "shmdata socket path to create",
						  NULL),
		    (Method::method_ptr) &add_file_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
		    true,
		    true,
		    this);
    
    publish_method ("Remove File",
		    "remove_file", 
		    "remove the file from the player", 
		    "success or fail",
		    Method::make_arg_description ("File Path",
						  "filepath", 
						  "file path to remove", 
						  NULL),
		    (Method::method_ptr) &remove_file_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    true,
		    true,
		    this);
    
    
//registering playing property
    playing_ = FALSE;
    custom_prop_.reset (new CustomPropertyHelper ());
    playing_param_ = custom_prop_->make_boolean_property ("playing", 
							  "start/stop playing",
							  FALSE,
							  (GParamFlags) G_PARAM_READWRITE,
							  ShmdataFromGDPFile::set_playing,
							  ShmdataFromGDPFile::get_playing,
							  this);
    register_property_by_pspec (custom_prop_->get_gobject (), 
     				playing_param_, 
     				"playing",
				"Playing",
				true,
				true);
    
    //set the name before registering properties
    set_name (g_strdup_printf ("gdpfilesrc%s", g_param_spec_get_name(playing_param_)));//FIXME implement and use make_name () in quiddity class, should be used for gsoap also
    return true;
  }

  gboolean
  ShmdataFromGDPFile::add_file_wrapped (gpointer shmdata_socket_path, 
					gpointer file_location,
					gpointer user_data)
  {
    ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile*>(user_data);
       
    if (context->add_file ((char *)shmdata_socket_path, (char *)file_location))
      return TRUE;
    else
      return FALSE;
  }

  bool
  ShmdataFromGDPFile::add_file (std::string file_path, 
				std::string shmdata_path	)
  {
    if (shmdata_names_.contains (file_path))
      {
	g_warning ("ShmdataFromGDPFile::add_file: %s is already added", file_path.c_str());
	return false;
      }
    
    shmdata_names_.insert(file_path, shmdata_path);
    return true;
  }  

  gboolean
  ShmdataFromGDPFile::remove_file_wrapped (gpointer file_path, 
					   gpointer user_data)
  {
    ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile*>(user_data);
    if (context->remove_file ((char *)file_path))
      return TRUE;
    else
      return FALSE;
  }

  bool
  ShmdataFromGDPFile::remove_file (std::string file_path)
  {
    if (!shmdata_names_.contains (file_path))
      return false;
    shmdata_names_.remove (file_path);
    return true;
  }

  void 
  ShmdataFromGDPFile::set_playing (gboolean playing, void *user_data)
  {
    ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile *> (user_data);

    if (playing)
      context->make_players ();
    else
      context->clean_players ();
    context->playing_ = playing;
    GObjectWrapper::notify_property_changed (context->custom_prop_->get_gobject (), context->playing_param_);
  }
  
  gboolean 
  ShmdataFromGDPFile::get_playing (void *user_data)
  {
    ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile *> (user_data);
    return context->playing_;
  }

  bool
  ShmdataFromGDPFile::make_players ()
  {
    if (!(bool) manager_) 
      {
	g_debug ("creating manager");
	manager_ = QuiddityManager::make_manager ("manager_"+get_name());
	manager_->create ("runtime","single_runtime");//only one runtime for all
	manager_->invoke_va("single_runtime", 
			    "pause", 
			    NULL, 
			    NULL);
      }
    std::map <std::string, std::string> shmdatas = shmdata_names_.get_map ();
    std::map <std::string, std::string>::iterator it;
    for (it = shmdatas.begin (); it != shmdatas.end (); it++)
      {
	manager_->create ("gstsrc", it->first.c_str ());
	manager_->invoke_va (it->first.c_str (), 
			     "set_runtime", 
			     NULL, 
			     "single_runtime", 
			     NULL);
	gchar *pipe = g_strdup_printf ("filesrc location=%s ! gdpdepay ! identity sync=true",
	 			       it->first.c_str ());
	g_debug ("ShmdataFromGDPFile::make_players %s", pipe); 
	manager_->invoke_va (it->first.c_str (),
			     "to_shmdata_with_path", 
			     NULL,
			     pipe, 
	 		     it->second.c_str (), 
	 		     NULL);
	g_free (pipe);
      }
    manager_->invoke_va("single_runtime", 
			"play", 
			NULL, 
			NULL);
    return true;
  }

  bool
  ShmdataFromGDPFile::clean_players ()
  {
    manager_.reset ();
    return true;
  }

  //FIXME use signals in switcher for handling gstsrc's eos 
  // void 
  // ShmdataFromGDPFile::rewind (gpointer user_data)
  // {
  //   ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile *>(user_data);
  //   context->set_playing (FALSE, context);
  // }
    
  // gboolean
  // ShmdataFromGDPFile::event_probe_cb (GstPad *pad, GstEvent * event, gpointer user_data)
  // {
  //   ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile *>(user_data);
  //   if (GST_EVENT_TYPE (event) == GST_EVENT_EOS) { 
  //     g_print ("EOS caught and disabled \n");
  //     g_print ("----- pad with EOS %s:%s, src: %p %s\n",
  //      	       GST_DEBUG_PAD_NAME (pad),GST_EVENT_SRC(event), gst_element_get_name (GST_EVENT_SRC(event)));
  //     g_idle_add ((GSourceFunc) ShmdataFromGDPFile::rewind,   
  //      		  (gpointer)context);   
  //     return FALSE;
  //   }  
  //   return TRUE; 
  // }


}
