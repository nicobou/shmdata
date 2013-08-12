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


/**
 * The Method class
 */

#include "method.h"

namespace switcher
{

  Method::Method () :
    closure_ (NULL)
  {
    json_description_.reset (new JSONBuilder());
    is_configuration_ = false;
    is_control_ = false;
  }

  Method::~Method ()
  {
    if (closure_ != NULL)
      g_closure_unref (closure_);
  }


  bool
  Method::set_method (method_ptr method, 
		      return_type return_type,
		      args_types arg_types, 
		      gpointer user_data)  
  {
    if (arg_types.size () < 1)
      {
	g_debug ("Method::set_method is called with empty arg_types");
	return false;
      }
    if (method == NULL)
      {
	g_debug ("Method::set_method is called with a NULL function");
	return false;
      }
    closure_ = g_cclosure_new (G_CALLBACK (method), user_data, Method::destroy_data);
    g_closure_set_marshal  (closure_,g_cclosure_marshal_generic);
    return_type_ =  return_type;
    arg_types_ = arg_types;
    num_of_value_args_ = arg_types_.size();

    return true;
  }


  //FIXME remove this method
  uint 
  Method::get_num_of_value_args()
  {
    return num_of_value_args_;
  }
  

  //FIXME should be commented and not used (using signal action instead)
  GValue 
  Method::invoke(std::vector<std::string> args)
  {
        
    GValue result_value = G_VALUE_INIT;

    if (args.size () != num_of_value_args_ && arg_types_[0] != G_TYPE_NONE)
      {
	g_warning ("Method::invoke number of arguments does not correspond to the size of argument types");
	return result_value;
      }

    GValue params[arg_types_.size ()];

    //with args
    if (arg_types_[0] != G_TYPE_NONE)
      for (gulong i=0; i < num_of_value_args_; i++)
	{
	  params[i] = G_VALUE_INIT;
	  g_value_init (&params[i],arg_types_[i]);
	  if (!gst_value_deserialize (&params[i],args[i].c_str()))
	    {
	      g_error ("Method::invoke string not transformable into gvalue (argument: %s) ",
		       args[i].c_str());
	      return result_value;
	    }
	}
    else
      {
	params[0] = G_VALUE_INIT;
	g_value_init (&params[0],G_TYPE_STRING);
	gst_value_deserialize (&params[0],"");
      }

    //gboolean result;
    g_value_init (&result_value, return_type_);
    //g_print ("arg tipe size %d\n", arg_types_.size());
    g_closure_invoke (closure_, &result_value, num_of_value_args_, params, NULL);
    
    //result = g_value_get_boolean (&result_value);
    
    //unset
    //g_value_unset (&result_value);
    for (guint i=0; i < num_of_value_args_; i++)
      g_value_unset (&params[i]);
    //return result;
    return result_value; 
  } 
  
  void
  Method::destroy_data (gpointer  data,
			GClosure *closure)
  {
    //g_debug ("Method::destroy data");
  }
  

  void
  Method::set_description (std::string long_name,
			   std::string method_name,
			   std::string short_description,
			   std::string return_description,
			   args_doc arg_description,
			   bool is_configuration,
			   bool is_control)
  {
    is_configuration_ = is_configuration;
    is_control_ = is_control;

    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("long name", long_name.c_str ());
    json_description_->add_string_member ("name", method_name.c_str ());
    json_description_->add_string_member ("description", short_description.c_str ());
    json_description_->add_string_member ("return type", g_type_name (return_type_));
    json_description_->add_string_member ("return description", return_description.c_str ());
    
    if (is_configuration_)
      json_description_->add_string_member ("is configuration", "true");
    else
      json_description_->add_string_member ("is configuration", "false");
    
    if (is_control_)
      json_description_->add_string_member ("is control", "true");
    else
      json_description_->add_string_member ("is control", "false");
    
    json_description_->set_member_name ("arguments");
    json_description_->begin_array ();
    args_doc::iterator it;
    int j=0;
    if (!arg_description.empty ())
      for (it = arg_description.begin() ; it != arg_description.end(); it++ )
	{
	  json_description_->begin_object ();
	  json_description_->add_string_member ("long name", std::get<0>(*it).c_str ());
	  json_description_->add_string_member ("name",std::get<1>(*it).c_str ());
	  json_description_->add_string_member ("description",std::get<2>(*it).c_str ());
	  json_description_->add_string_member ("type",g_type_name (arg_types_[j])); 
	  json_description_->end_object ();
	}
    json_description_->end_array ();
    json_description_->end_object ();
  }

  //json formated description
  std::string
  Method::get_description ()
  {
    return json_description_->get_string (true);
  }

  JSONBuilder::Node 
  Method::get_json_root_node ()
  {
    return json_description_->get_root ();
  }

  
   std::vector<GType> 
   Method::make_arg_type_description (GType first_arg_type, ...)
   {
     std::vector<GType> res;
     GType arg_type;
     va_list vl;
     va_start(vl, first_arg_type);
     res.push_back (first_arg_type);
     while ( (arg_type = va_arg(vl, GType)))
       res.push_back (arg_type);
     va_end(vl);
     return res;
   }


  //FIXME, make this more robust to user missing strings
  Method::args_doc
  Method::make_arg_description (const char *first_arg_long_name, ...)
  {
    args_doc res;
    va_list vl;
    char *arg_long_name;
    char *arg_name;
    char *arg_desc;
    va_start(vl, first_arg_long_name);
    if (g_strcmp0 (first_arg_long_name, "none") != 0 
	&& (arg_name = va_arg(vl, char *)) 
	&& (arg_desc = va_arg(vl, char *)))
      res.push_back (std::make_tuple (first_arg_long_name, 
				      arg_name,
				      arg_desc));
    gboolean parsing = true;
    do
      {
	arg_long_name = va_arg( vl, char *);
	
	if (arg_long_name != NULL)
	  {
	    arg_name = va_arg( vl, char *); 
	    arg_desc = va_arg( vl, char *);
	    if (arg_name != NULL && arg_desc != NULL)
	      res.push_back (std::make_tuple (arg_long_name, 
					      arg_name,
					      arg_desc));
	    else
	      parsing = false;
	  }
	else
	  parsing = false;
      }
    while (parsing);
    va_end(vl);
    return res;
  }

}
