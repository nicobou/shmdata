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

#include "segment.h"
#include "gst-utils.h"

namespace switcher
{

  GParamSpec *Segment::json_writers_description_ = NULL;
  GParamSpec *Segment::json_readers_description_ = NULL;

  Segment::Segment()
  {
    shmdata_writers_description_.reset (new JSONBuilder());
    shmdata_readers_description_.reset (new JSONBuilder());
    update_shmdata_writers_description ();
    update_shmdata_readers_description ();

    //installing custom prop for json shmdata description
    if (json_writers_description_ == NULL)
      json_writers_description_ = 
	GObjectWrapper::make_string_property ("shmdata-writers", 
					      "json formated shmdata writers description",
					      "",
					      (GParamFlags) G_PARAM_READABLE,
					      NULL,
					      Segment::get_shmdata_writers_by_gvalue);
     
    if (json_readers_description_ == NULL)
      json_readers_description_ = 
	GObjectWrapper::make_string_property ("shmdata-readers", 
					      "json formated shmdata readers description",
					      "",
					      (GParamFlags) G_PARAM_READABLE,
					      NULL,
					      Segment::get_shmdata_readers_by_gvalue);

    install_property_by_pspec (gobject_->get_gobject (), 
				json_writers_description_, 
				"shmdata-writers",
				"Shmdata Writers");
    install_property_by_pspec (gobject_->get_gobject (), 
				json_readers_description_, 
				"shmdata-readers",
				"Shmdata Readers");

    make_bin();

    install_method ("Set Runtime", 
		    "set_runtime",
		    "attach a quiddity/segment to a runtime",
		    "success or fail",
		    Method::make_arg_description ("Runtime Name",
						  "runtime_name",
						  "the name of the runtime quiddity to attach with",
						  NULL),
		    (Method::method_ptr) &set_runtime_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL), 
		    this);


    //FIXME
     // GType types[] = {G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING};
     // make_custom_signal_with_class_name ("segment",
     // 					 "on-new-shmdata-writer", 
     // 					 G_TYPE_NONE,
     // 					 3,
     // 					 types);
     // set_signal_description ("On New Shmdata Writer",
     // 			     "on-new-shmdata-writer",
     // 			     "a new shmdata writer has been created",
     // 			     Signal::make_arg_description("Quiddity Name",
     // 							  "quiddity_name",
     // 							  "the quiddity name",
     // 							  "Path",
     // 							  "path",
     // 							  "the shmdata path",
     // 							  "JSON Documentation",
     // 							  "json_doc",
     // 							  "the writer json documentation",
     // 							  NULL));
     // make_custom_signal_with_class_name ("segment",
     // 					 "on-new-shmdata-reader", 
     // 					 G_TYPE_NONE,
     // 					 3,
     // 					 types);
     // set_signal_description ("On New Shmdata Reader",
     // 			     "on-new-shmdata-reader",
     // 			     "a new shmdata reader has been created",
     // 			     Signal::make_arg_description("Quiddity Name",
     // 							  "quiddity_name",
     // 							  "the quiddity name",
     // 							  "Path",
     // 							  "path",
     // 							  "the shmdata path",
     // 							  "JSON Documentation",
     // 							  "json_doc",
     // 							  "the writer json documentation",
     // 							  NULL));
  }

  Segment::~Segment()
  {
    g_debug ("Segment::~Segment begin (%s)",get_nick_name().c_str ());
    clean_bin ();
    g_debug ("Segment::~Segment end");
  }
  
  void 
  Segment::make_bin ()
  {
    GstUtils::make_element ("bin", &bin_);
    g_object_set (G_OBJECT (bin_), "async-handling",TRUE, NULL);

  }
  
  void
  Segment::clean_bin()
  {
    g_debug ("Segment, bin state %s, target %s, num children %d ", 
	     gst_element_state_get_name (GST_STATE (bin_)), 
	     gst_element_state_get_name (GST_STATE_TARGET (bin_)), 
	     GST_BIN_NUMCHILDREN(GST_BIN (bin_)));
    
    GstUtils::wait_state_changed (bin_);
    
    if (GST_IS_ELEMENT (bin_))
      {
	clear_shmdatas ();

	g_debug ("Segment, bin state %s, target %s, num children %d ", 
		 gst_element_state_get_name (GST_STATE (bin_)), 
		 gst_element_state_get_name (GST_STATE_TARGET (bin_)), 
		 GST_BIN_NUMCHILDREN(GST_BIN (bin_)));
	
	if (GST_BIN_CHILDREN (bin_) > 0)
	  {
	    g_debug ("segment: some child elements have not been cleaned in %s",
		     get_nick_name ().c_str ());
	    GList *child = NULL, *children = GST_BIN_CHILDREN (bin_);
	    for (child = children; child != NULL; child = g_list_next (child)) 
	      {
		g_debug ("segment warning: child %s", GST_ELEMENT_NAME (GST_ELEMENT (child->data)));
		//GstUtils::clean_element (GST_ELEMENT (child->data));
	      }
	  }
	g_debug ("~Segment: cleaning internal bin");
	GstUtils::clean_element (bin_);
      }
  }

  void 
  Segment::set_runtime_wrapped (gpointer arg, gpointer user_data)
  {
    gchar *runtime_name = (gchar *)arg;
    Segment *context = static_cast<Segment*>(user_data);
    
    if (runtime_name == NULL) 
      {
	g_debug ("Segment::set_runtime_wrapped Error: runtime_name is NULL");
	return;
      }
    if (context == NULL) 
      {
	g_debug ("Segment::set_runtime_wrapped Error: segment is NULL");
	return;
      }
    
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    if ( (bool)manager)
      {
	Quiddity::ptr quidd = manager->get_quiddity (runtime_name);
	Runtime::ptr runtime = std::dynamic_pointer_cast<Runtime> (quidd);
	if(runtime)
	  context->set_runtime(runtime);
	else
	  g_warning ("Segment::set_runtime_wrapped Error: %s is not a runtime",runtime_name);
      }
    //g_debug ("%s is attached to runtime %s",context->get_name().c_str(),runtime->get_name().c_str());
  }
  
  void
  Segment::set_runtime (Runtime::ptr runtime)
  {
    if (runtime_ != NULL)
      {
	clean_bin ();
	make_bin ();
      }

    runtime_ = runtime;
    gst_bin_add (GST_BIN (runtime_->get_pipeline ()),bin_);

    //start the shmdata reader
    std::vector<ShmdataReader::ptr> shmreaders = shmdata_readers_.get_values ();
    std::vector<ShmdataReader::ptr>::iterator it;
    for (it = shmreaders.begin (); it != shmreaders.end (); it++)
      	(*it)->start ();

    //GstUtils::wait_state_changed (runtime_->get_pipeline ());
    GstUtils::sync_state_with_parent (bin_);
    GstUtils::wait_state_changed (bin_);
    //g_debug ("Segment::set_runtime (done), %s", gst_element_state_get_name (GST_STATE (bin_)));
  }
  
  GstElement *
  Segment::get_bin()
  {
    return bin_;
  }

  void
  Segment::update_shmdata_writers_description ()
  {
    shmdata_writers_description_->reset();
    shmdata_writers_description_->begin_object ();
    shmdata_writers_description_->set_member_name ("shmdata_writers");
    shmdata_writers_description_->begin_array ();

    std::vector<ShmdataWriter::ptr> shmwriters = shmdata_writers_.get_values ();
    std::vector<ShmdataWriter::ptr>::iterator it;
    if (shmwriters.begin () != shmwriters.end ())
      for (it = shmwriters.begin (); it != shmwriters.end (); it++)
	shmdata_writers_description_->add_node_value ( (*it)->get_json_root_node ());
    
    shmdata_writers_description_->end_array ();
    shmdata_writers_description_->end_object ();
  }

  void
  Segment::update_shmdata_readers_description ()
  {
    shmdata_readers_description_->reset();
    shmdata_readers_description_->begin_object ();
    shmdata_readers_description_->set_member_name ("shmdata_readers");
    shmdata_readers_description_->begin_array ();

    std::vector<ShmdataReader::ptr> shmreaders = shmdata_readers_.get_values ();
    std::vector<ShmdataReader::ptr>::iterator it;
    if (shmreaders.begin () != shmreaders.end ())
      for (it = shmreaders.begin (); it != shmreaders.end (); it++)
	shmdata_readers_description_->add_node_value ( (*it)->get_json_root_node ());
    
    shmdata_readers_description_->end_array ();
    shmdata_readers_description_->end_object ();
  }

  bool 
  Segment::register_shmdata_writer (ShmdataWriter::ptr writer)
  {
    const gchar *name = writer->get_path ().c_str ();
    if (g_strcmp0 (name, "") == 0)
      {
	g_warning ("Segment:: can not register shmdata writer with no path");
	return false;
      }
    shmdata_writers_.insert (name, writer);
    update_shmdata_writers_description ();
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_writers_description_);
    signal_emit ("on-new-shmdata-writer", 
      		 get_nick_name ().c_str (), 
      		 writer->get_path ().c_str (),
      		 (JSONBuilder::get_string (writer->get_json_root_node (), true)).c_str ());
    return true;
  }
  
  bool Segment::register_shmdata_reader (ShmdataReader::ptr reader)
  {
    const gchar *name = reader->get_path ().c_str ();
    if (g_strcmp0 (name, "") == 0)
      {
	g_warning ("Segment:: can not register shmdata reader with no path");
	return false;
      }
    shmdata_readers_.insert (name, reader);
    update_shmdata_readers_description ();
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_readers_description_);
    signal_emit ("on-new-shmdata-reader", 
		 get_nick_name ().c_str (), 
		 reader->get_path ().c_str (),
		 JSONBuilder::get_string (reader->get_json_root_node (), true).c_str ());
    return true;
    
  }

  bool Segment::unregister_shmdata_reader (std::string shmdata_path)
  {
    shmdata_readers_.remove (shmdata_path);
    update_shmdata_readers_description ();
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_readers_description_);
    return true;
  }

  bool Segment::unregister_shmdata_writer (std::string shmdata_path)
  {
    shmdata_writers_.remove (shmdata_path);
    update_shmdata_writers_description ();
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_writers_description_);
    return true;
  }

  bool Segment::clear_shmdatas ()
  {
    unsigned int size = shmdata_writers_.size ();
    shmdata_writers_.clear ();
    if (size != 0)
      {
	update_shmdata_writers_description ();
	GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_writers_description_);
      }

    size = shmdata_readers_.size ();
    shmdata_readers_.clear ();
    if (size != 0)
      {
	update_shmdata_readers_description ();
	GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_readers_description_);
      }
    return true;
  }

  bool
  Segment::get_shmdata_writers_by_gvalue (GValue *value,
                                          void *user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    g_value_set_string (value, context->shmdata_writers_description_->get_string (true).c_str ());
    return TRUE;
  }

  bool
  Segment::get_shmdata_readers_by_gvalue (GValue *value,
                                          void *user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    g_value_set_string (value, context->shmdata_readers_description_->get_string (true).c_str ());
    return TRUE;
  }

  bool 
  Segment::reset_bin ()
  {
    clean_bin ();
    make_bin ();
    if (!(bool)runtime_)
      return false;
    set_runtime (runtime_);
    return true;
 }

}
