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
  }

  Method::~Method ()
  {
    if (closure_ != NULL)
      g_closure_unref (closure_);
  }

  /*
   * the first elements of arg_type should be args passed by value, 
   * with args passed by pointers at the end (as references to base quiddities)  
   *
   */
  void //TODO make set_method returning a bool
  Method::set_method (void *method, std::vector<GType> arg_types, gpointer user_data)  
  {
    if (arg_types.size () < 1)
      {
	g_warning ("Method::set_method is called with empty arg_types");
	return;
      }
    if (method == NULL)
      {
	g_warning ("Method::set_method is called with a NULL function");
	return;
      }
    closure_ = g_cclosure_new (G_CALLBACK (method), user_data, Method::destroy_data);
    g_closure_set_marshal  (closure_,g_cclosure_marshal_generic);
    arg_types_ = arg_types;
    num_of_value_args_ = arg_types_.size();
  }

  //FIXME remove this method
  uint 
  Method::get_num_of_value_args()
  {
    return num_of_value_args_;
  }
  
  bool 
  Method::invoke(std::vector<std::string> args)
  {
    if (args.size () != num_of_value_args_ && arg_types_[0] != G_TYPE_NONE)
      {
	g_warning ("Method::invoke number of arguments does not correspond to the size of argument types");
	return false;
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
	      return false;
	    }
	}
    else
      {
	params[0] = G_VALUE_INIT;
	g_value_init (&params[0],G_TYPE_STRING);
	gst_value_deserialize (&params[0],"");
      }

    
    GValue result_value = G_VALUE_INIT;
    gboolean result;
    g_value_init (&result_value, G_TYPE_BOOLEAN);
    //g_print ("arg tipe size %d\n", arg_types_.size());
    g_closure_invoke (closure_, &result_value, num_of_value_args_, params, NULL);
    result = g_value_get_boolean (&result_value);
    
    //unset
    g_value_unset (&result_value);
    for (guint i=0; i < num_of_value_args_; i++)
      g_value_unset (&params[i]);
    return result;
  } 
  
  void
  Method::destroy_data (gpointer  data,
			GClosure *closure)
  {
    //g_debug ("Method::destroy data");
  }
  

  void
  Method::set_description (std::string method_name,
			   std::string short_description,
			   std::vector< std::pair<std::string,std::string> > arg_description)
  {
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("name", method_name.c_str ());
    json_description_->add_string_member ("description", short_description.c_str ());
    json_description_->set_member_name ("arguments");
    json_description_->begin_array ();
    std::vector<std::pair<std::string,std::string> >::iterator it;
    int j=0;
    if (!arg_description.empty ())
      for (it = arg_description.begin() ; it != arg_description.end(); it++ )
	{
	  json_description_->begin_object ();
	  json_description_->add_string_member ("name",it->first.c_str ());
	  json_description_->add_string_member ("description",it->second.c_str ());
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
     while (arg_type = va_arg( vl, GType))
       res.push_back (arg_type);
     va_end(vl);
     return res;
   }

  std::vector<std::pair<std::string,std::string> > 
  Method::make_arg_description (char *first_arg_name, ...)
  {
    std::vector<std::pair<std::string,std::string> > res;
    std::pair<std::string,std::string> arg_desc_pair;
    va_list vl;
    char *arg_name;
    char *arg_desc;
    va_start(vl, first_arg_name);
    if (first_arg_name != "none" && (arg_desc = va_arg( vl, char *)))
      {
	std::pair<std::string,std::string> arg_pair;
	arg_desc_pair.first = std::string (first_arg_name);
	arg_desc_pair.second = std::string (arg_desc);
	res.push_back (arg_desc_pair);
      }
    while ( (arg_name = va_arg( vl, char *)) && (arg_desc = va_arg( vl, char *)))
      {
	std::pair<std::string,std::string> arg_pair;
	arg_desc_pair.first = std::string (arg_name);
	arg_desc_pair.second = std::string (arg_desc);
	res.push_back (arg_desc_pair);
      }
    
    va_end(vl);
    return res;
  }
  
}
