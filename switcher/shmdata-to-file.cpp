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

#include "shmdata-to-file.h"
#include <glib/gstdio.h> 
#include "gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataToFile,
				       "Shmdata Recorder",
				       "file recorder", 
				       "record shmdata(s) to file(s)",
				       "LGPL",
				       "shmtofile",
				       "Nicolas Bouillot");
  
  ShmdataToFile::ShmdataToFile () :
    custom_prop_ (new CustomPropertyHelper ()),  
    recording_param_ (NULL), 
    recording_ (FALSE), 
    file_names_ (),
    shmdata_recorders_ ()
  {}

  ShmdataToFile::~ShmdataToFile ()
  {
    clean_recorders ();
  }

  bool 
  ShmdataToFile::init_segment ()
  {
    install_method ("Add Shmdata",
		    "add_shmdata", 
		    "add a shmdata to record", 
		    "success or fail",
		    Method::make_arg_description ("Shmdata Path",
						  "shmpath", 
						  "shmdata socket path to record",
						  "File Path"
						  "filepath",
						  "file location",
						  NULL),
		    (Method::method_ptr) &add_shmdata_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
		    this);
    
    install_method ("Remove Shmdata",
		    "remove_shmdata", 
		    "remove a shmdata from the recorder", 
		    "success or fail",
		    Method::make_arg_description ("Shmdata Path",
						  "shmpath", 
						  "shmdata socket path to remove", 
						  NULL),
		    (Method::method_ptr) &remove_shmdata_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);
    
 
    //registering recording property
    recording_param_ = custom_prop_->make_boolean_property ("recording", 
							    "start/stop recording",
							    FALSE,
							    (GParamFlags) G_PARAM_READWRITE,
							    ShmdataToFile::set_recording,
							    ShmdataToFile::get_recording,
							    this);
    install_property_by_pspec (custom_prop_->get_gobject (), 
     				recording_param_, 
     				"recording",
				"Recording");
    return true;
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
    if (file_names_.find (shmdata_socket_path) != file_names_.end ())
      {
	g_warning ("ShmdataToFile::add_shmdata: %s is already added", 
		   shmdata_socket_path.c_str());
	return false;
      }

    file_names_[shmdata_socket_path] = file_location;
    
     if (recording_) // starting the reader if runtime is set
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
    for (auto &it :file_names_)
      {
	//FIXME check file
	GError *error = NULL;
	gchar *pipe = g_strdup_printf ("gdppay ! filesink location=%s",
				       it.second.c_str());
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
	 //GstUtils::wait_state_changed (bin_);
	 GstUtils::sync_state_with_parent (recorder_bin);
	 
	 ShmdataReader::ptr reader;
	 reader.reset (new ShmdataReader ());
	 reader->set_path (it.first.c_str());
	 reader->set_bin (bin_);
	 reader->set_g_main_context (get_g_main_context ());
	 reader->set_sink_element (recorder_bin);
	 
	 //GstUtils::wait_state_changed (bin_);
	 reader->start ();
	 shmdata_recorders_[it.first] = recorder_bin;
	 register_shmdata_reader (reader);
      }
    return true;
  }

  bool
  ShmdataToFile::clean_recorders ()
  {
   
    for (auto &it : shmdata_recorders_)
      {
	GstUtils::clean_element (it.second);
	unregister_shmdata_reader (it.first);
      }
    shmdata_recorders_.clear ();
    return true;
  }
}
