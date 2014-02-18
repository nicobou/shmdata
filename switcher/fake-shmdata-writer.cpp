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

#include "fake-shmdata-writer.h"
#include "gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(FakeShmdataWriter,
				       "Shmdata From Software",
				       "fake source", 
				       "add a shmdata from an other software",
				       "LGPL",
				       "fakeshmsrc",
				       "Nicolas Bouillot");
  
  FakeShmdataWriter::FakeShmdataWriter () :
    custom_props_ (new CustomPropertyHelper ()),
    shmdata_path_spec_ (NULL),
    shmdata_path_ (g_strdup ("none"))
  {}

  bool
  FakeShmdataWriter::init_segment ()
  {
    init_startable (this);

    shmdata_path_ = g_strdup ("none");
    custom_props_.reset (new CustomPropertyHelper ());
    shmdata_path_spec_ = 
      custom_props_->make_string_property ("shmdata-path", 
					   "Path Of The Shmdata The Include",
					   "",
					   (GParamFlags) G_PARAM_READWRITE,
					   FakeShmdataWriter::set_shmdata_path,
					   FakeShmdataWriter::get_shmdata_path,
					   this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				shmdata_path_spec_, 
				"shmdata-path",
				"Shmdata Path");

    return true;
  }
  
  bool
  FakeShmdataWriter::add_shmdata_path (std::string name)
  {
    //creating a shmdata
    ShmdataWriter::ptr connector;
    connector.reset (new ShmdataWriter ());
    connector->set_path_without_deleting (name.c_str());
    register_shmdata_writer (connector);

    g_message ("%s created a new shmdata writer (%s)", 
     	       get_nick_name ().c_str(), 
     	       name.c_str ());
    return true;
  }
  
  void 
  FakeShmdataWriter::set_shmdata_path (const gchar *value, void *user_data)
  {
    FakeShmdataWriter *context = static_cast <FakeShmdataWriter *> (user_data);
    g_free (context->shmdata_path_);
    
    context->shmdata_path_ = g_strdup (value);
    context->custom_props_->notify_property_changed (context->shmdata_path_spec_);
  }
  
  const gchar *
  FakeShmdataWriter::get_shmdata_path (void *user_data)
  {
    FakeShmdataWriter *context = static_cast <FakeShmdataWriter *> (user_data);
    return context->shmdata_path_;
  }

  FakeShmdataWriter::~FakeShmdataWriter ()
  {
    g_free (shmdata_path_);
  }

  bool
  FakeShmdataWriter::clean ()
  {
    return unregister_shmdata_writer (shmdata_path_);
  }

  bool 
  FakeShmdataWriter::start ()
  {
    clean ();
    uninstall_property ("shmdata-path");
    if (g_strcmp0 (shmdata_path_, "") == 0)
      return false;
    
    //creating a shmdata
    ShmdataWriter::ptr connector;
    connector.reset (new ShmdataWriter ());
    connector->set_path_without_deleting (shmdata_path_);
    register_shmdata_writer (connector);

    g_message ("%s created a new shmdata writer (%s)", 
     	       get_nick_name ().c_str(), 
     	       shmdata_path_);
 
    return true;
  }

  bool 
  FakeShmdataWriter::stop ()
  {
    clean ();
    install_property_by_pspec (custom_props_->get_gobject (), 
				shmdata_path_spec_, 
				"shmdata-path",
				"Shmdata Path");
    return true;
  }
}
