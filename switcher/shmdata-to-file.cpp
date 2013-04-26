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

#include "shmdata-to-file.h"
#include <glib/gstdio.h> 
#include "gst-utils.h"

namespace switcher
{
  QuiddityDocumentation ShmdataToFile::doc_ ("file sink", "shmtofile",
					     "record shmdata(s) to file(s)");
  
  ShmdataToFile::~ShmdataToFile ()
  {
    clean_recorders ();
  }

  bool 
  ShmdataToFile::init ()
  {
    //registering add_shmdata
    register_method("add_shmdata",
		    (void *)&add_shmdata_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("add_shmdata", 
			    "add a shmdata to record", 
			    Method::make_arg_description ((char *)"shmpath", 
							  (char *)"shmdata socket path to record",
							  (char *)"filepath",
							  (char *)"file location",
							  NULL));

    //registering remove_shmdata
    register_method("remove_shmdata", 
		    (void *)&remove_shmdata_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ((char *)"remove_shmdata", 
			    (char *)"remove a shmdata from the recorder", 
			    Method::make_arg_description ((char *)"shmpath", 
							  (char *)"shmdata socket path to remove", 
							  NULL));
    
    //registering recording property
    recording_ = FALSE;
    custom_prop_.reset (new CustomPropertyHelper ());
    recording_param_ = custom_prop_->make_boolean_property ("recording", 
							    "start/stop recording",
							    FALSE,
							    (GParamFlags) G_PARAM_READWRITE,
							    ShmdataToFile::set_recording,
							    ShmdataToFile::get_recording,
							    this);
    register_property_by_pspec (custom_prop_->get_gobject (), 
     				recording_param_, 
     				"recording");

    //set the name before registering properties
    set_name (gst_element_get_name (bin_));
    return true;
  }
  
  QuiddityDocumentation 
  ShmdataToFile::get_documentation ()
  {
    return doc_;
  }

  gboolean
  ShmdataToFile::add_shmdata_wrapped (gpointer shmdata_socket_path, 
					  gpointer file_location,
					  gpointer user_data)
  {
    ShmdataToFile *context = static_cast<ShmdataToFile*>(user_data);
       
    if (context->add_shmdata ((char *)shmdata_socket_path, (char *)file_location))
      return TRUE;
    else
      return FALSE;
  }

  bool
    ShmdataToFile::add_shmdata (std::string shmdata_socket_path, 
				std::string file_location)
  {
    if (file_names_.contains (shmdata_socket_path))
      {
	g_warning ("ShmdataToFile::add_shmdata: %s is already added", shmdata_socket_path.c_str());
	return false;
      }

    file_names_.insert(shmdata_socket_path, file_location);
    
     if (recording_ && runtime_) // starting the reader if runtime is set
       {
	     //FIXME make the recorder
       }
     
    return true;
  }  

  gboolean
  ShmdataToFile::remove_shmdata_wrapped (gpointer connector_name, 
					     gpointer user_data)
  {
    ShmdataToFile *context = static_cast<ShmdataToFile*>(user_data);
    if (context->remove_shmdata ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  ShmdataToFile::remove_shmdata (std::string shmdata_socket_path)
  {
    unregister_shmdata_reader (shmdata_socket_path);
    g_debug ("data_stream %s removed", shmdata_socket_path.c_str ());
    return true;
  }

  void 
  ShmdataToFile::set_recording (gboolean recording, void *user_data)
  {
    ShmdataToFile *context = static_cast<ShmdataToFile *> (user_data);
    if (recording)
      context->make_recorders ();
    else
      context->clean_recorders ();
    context->recording_ = recording;
  }
  
  gboolean 
  ShmdataToFile::get_recording (void *user_data)
  {
    ShmdataToFile *context = static_cast<ShmdataToFile *> (user_data);
    return context->recording_;
  }


  bool
  ShmdataToFile::make_recorders ()
  {
    std::map <std::string, std::string> files = file_names_.get_map ();
    std::map <std::string, std::string>::iterator it;
    for (it = files.begin (); it != files.end (); it++)
      {
	//FIXME check file
	 GError *error = NULL;
	 gchar *pipe = g_strdup_printf ("gdppay ! filesink location=%s",it->second.c_str());
	 GstElement *recorder_bin = gst_parse_bin_from_description (pipe,
								    TRUE,
								    &error);
	 g_free (pipe);
	 if (error != NULL)
	   {
	     g_warning ("%s",error->message);
	     g_error_free (error);
	     return false;
	   }
	 gst_bin_add (GST_BIN (bin_), recorder_bin);
	 GstUtils::wait_state_changed (bin_);
	 GstUtils::sync_state_with_parent (recorder_bin);

	 ShmdataReader::ptr reader;
	 reader.reset (new ShmdataReader ());
	 reader->set_path (it->first.c_str());
	 reader->set_bin (bin_);
	 reader->set_sink_element (recorder_bin);
	
	 GstUtils::wait_state_changed (bin_);
	 reader->start ();
	 shmdata_recorders_.insert(it->first, recorder_bin);
	  
	 register_shmdata_reader (reader);
	
      }

  }

  bool
  ShmdataToFile::clean_recorders ()
  {
    
    std::map <std::string, GstElement *> recorders = shmdata_recorders_.get_map ();
    std::map <std::string, GstElement *>::iterator it;
    for (it = recorders.begin (); it != recorders.end (); it++)
      {
	GstUtils::clean_element (it->second);
	unregister_shmdata_reader (it->first);
      }
    shmdata_recorders_.clear ();
  }
}
