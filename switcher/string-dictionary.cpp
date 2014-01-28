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

#include "string-dictionary.h"
#include "property.h"
#include <string.h>
#include <iostream>

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(StringDictionary,
				       "Dictionary",
				       "dictionary", 
				       "Dictionary of string key/values accessible through properties",
				       "LGPL",
				       "dico",
				       "Nicolas Bouillot");
  StringDictionary::StringDictionary() :
    dico_ (),
    set_get_contexts_ (),
    custom_props_ (new CustomPropertyHelper ()),
    prop_specs_ ()
  {}

  StringDictionary::~StringDictionary()
  {}
    
  bool
  StringDictionary::init()
  {
    install_method ("Create A New Entry",
		    "new-entry",
		    "create a new dictionary entry accessible through a property sharing the same name",
		    "success or failure",
		    Method::make_arg_description ("Entry Name (Key)",
						  "name", 
						  "string",
						  "Entry Description",
						  "description", 
						  "string",
						  "Long Name",
						  "long-name", 
						  "string",
						  NULL),
  		    (Method::method_ptr) &create_entry, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, NULL),
		    this);

    install_method ("Remove An Entry",
		    "remove-entry",
		    "remove the entry and its property",
		    "success or failure",
		    Method::make_arg_description ("Entry Name",
						  "name", 
						  "string",
						  NULL),
  		    (Method::method_ptr) &remove_entry, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);

    install_method ("Save To A File",
     		    "save",
     		    "save dictionary to a file",
     		    "success or failure",
     		    Method::make_arg_description ("File path",
     						  "path", 
     						  "string",
     						  NULL),
     		    (Method::method_ptr) &save, 
     		    G_TYPE_BOOLEAN,
     		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
     		    this);
    
    install_method ("Load From File",
     		    "load",
     		    "Load dictionary from a file",
     		    "success or failure",
     		    Method::make_arg_description ("File path",
     						  "path", 
     						  "string",
     						  NULL),
     		    (Method::method_ptr) &load, 
     		    G_TYPE_BOOLEAN,
     		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
     		    this);
  
    return true;
  }

  gboolean
  StringDictionary::create_entry (const gchar *entry_name, 
				  const gchar *entry_description, 
				  const gchar *long_name,
				  void *user_data)
  {
    StringDictionary *context = static_cast<StringDictionary *> (user_data);
    
    if (context->dico_.find (entry_name) != context->dico_.end ())
      {
	g_debug ("cannot create entry named %s (already existing)", entry_name);
	return FALSE;
      }
    
    std::shared_ptr <PropertySetGet> prop_context;
    prop_context.reset (new PropertySetGet ()); 
    prop_context->string_dictionary = context;
    prop_context->entry_name = entry_name;
    context->set_get_contexts_[entry_name] = prop_context;
    
    context->dico_[entry_name] = g_strdup ("");

    context->prop_specs_[entry_name] = 
      context->custom_props_->make_string_property (entry_name,  
						    entry_description, 
						    "",
						    (GParamFlags) G_PARAM_READWRITE,  
						    StringDictionary::string_setter,
						    StringDictionary::string_getter,
						      prop_context.get ()); 
    
    context->install_property_by_pspec (context->custom_props_->get_gobject (), 
					 context->prop_specs_[entry_name], 
					 entry_name,
					 long_name);

    return TRUE;
  }

  gboolean
  StringDictionary::remove_entry (const gchar *entry_name, void *user_data)
  {
    StringDictionary *context = static_cast<StringDictionary *> (user_data);
    if (context->dico_.find (entry_name) == context->dico_.end ())
      {
	g_debug ("cannot remove entry named %s (not existing)", entry_name);
	return FALSE;
      }
    
    context->uninstall_property (entry_name);
    context->dico_.erase (entry_name);
    context->prop_specs_.erase (entry_name);
    context->set_get_contexts_.erase (entry_name);
    return TRUE;
  }

  gchar *
  StringDictionary::string_getter (void *user_data)
  {
    PropertySetGet *context = static_cast <PropertySetGet *> (user_data);
    return context->string_dictionary->dico_[context->entry_name];
  }
  
  void 
  StringDictionary::string_setter (const gchar *value, void *user_data)
  {
    PropertySetGet *context = static_cast <PropertySetGet *> (user_data);
    g_free (context->string_dictionary->dico_[context->entry_name]);
    context->string_dictionary->dico_[context->entry_name] = g_strdup (value);
    GObjectWrapper::notify_property_changed (context->string_dictionary->gobject_->get_gobject (), 
					     context->string_dictionary->prop_specs_[context->entry_name]);
  }


  gboolean
  StringDictionary::save_file (const gchar *file_path)
  {
    GFile *file = g_file_new_for_commandline_arg (file_path);
    GError *error = NULL;
    GFileOutputStream *file_stream = g_file_replace (file,
						     NULL,
						     TRUE, //make backup
						     G_FILE_CREATE_NONE ,
						     NULL,
						     &error);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return FALSE;
      }

    JSONBuilder::ptr builder;
    builder.reset (new JSONBuilder ());
    builder->reset ();
    //builder->begin_object ();
    //builder->set_member_name ("dictionary");
    builder->begin_array ();

    for (auto &it: dico_)
      {
	builder->begin_object ();
	Property::ptr prop = get_property_ptr (it.first);
	builder->add_string_member ("name", it.first.c_str ());
	builder->add_string_member ("description", prop->get_short_description ().c_str ());
	builder->add_string_member ("long name", prop->get_long_name ().c_str ());	
	builder->add_string_member ("value", it.second);
	builder->end_object ();
      }

    builder->end_array ();
    //builder->end_object ();
    
    gchar *dico_json = g_strdup (builder->get_string(true).c_str ());

    g_output_stream_write ((GOutputStream *)file_stream,
			   dico_json,
			   sizeof (gchar) * strlen (dico_json),
			   NULL,
			   &error);
    g_free (dico_json);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return FALSE;
      }
     
    g_output_stream_close ((GOutputStream *)file_stream,
			   NULL,
			   &error);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return FALSE;
      }
    
    g_object_unref(file_stream);
    return TRUE;
  }
  
  gboolean 
  StringDictionary::save (gchar *file_path, 
			  void *user_data)
  {
    StringDictionary *context = static_cast <StringDictionary *> (user_data);
    return context->save_file (file_path);
  }

  gboolean 
  StringDictionary::load_file (const gchar *file_path)
  {
    JsonParser *parser = json_parser_new ();
    GError *error = NULL;
    json_parser_load_from_file (parser,
				file_path,
				&error);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_object_unref(parser);
	g_error_free (error);
	return false;
      }
    
    JsonNode *root_node = json_parser_get_root (parser);
    JsonReader *reader = json_reader_new (root_node);
    
    if (!json_reader_is_array (reader))
      {
	g_debug ("malformed dictionary file");
	return false;
      }

    gint num_elements = json_reader_count_elements (reader);
    int i;
    for (i = 0; i < num_elements; i++)
      {
	json_reader_read_element (reader, i);

	json_reader_read_member (reader, "name");
	const gchar *name = json_reader_get_string_value (reader);
	json_reader_end_member (reader);

	json_reader_read_member (reader, "description");
	const gchar *description = json_reader_get_string_value (reader);
	json_reader_end_member (reader);

	json_reader_read_member (reader, "long name");
	const gchar *long_name = json_reader_get_string_value (reader);
	json_reader_end_member (reader);

	json_reader_read_member (reader, "value");
	const gchar *value = json_reader_get_string_value (reader);
	json_reader_end_member (reader);

	json_reader_end_element (reader);
	// g_print ("%s, %s, %s, %s\n",
	// 	 name, 
	// 	 description, 
	// 	 long_name, 
	// 	 value);
	if (create_entry (name, description, long_name, this))
	  {
	    g_free (dico_[name]);
	    dico_[name] = g_strdup (value);
	  }
      }
   
    g_object_unref(reader);
    g_object_unref(parser);

    return TRUE;
  }

  gboolean 
  StringDictionary::load (gchar *file_path, 
			  void *user_data)
  {
    StringDictionary *context = static_cast <StringDictionary *> (user_data);
    return context->load_file (file_path);
  }
  
}
