/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/method.h"
namespace switcher
{

  Method::Method () :
    closure_ (NULL)
  {}

  Method::~Method ()
  {
    if (closure_ != NULL)
      g_closure_unref (closure_);
  }

  /*
   * the first elements of arg_type should be args passed by value, 
   * with args passed by pointers at the end (as references to base entities)  
   *
   */
  void 
  Method::set_method (void *method, std::vector<GType> arg_types, gpointer user_data)  
  {
    if (method == NULL)
      {
	g_printerr ("Method::set_method is call with a NULL function\n");
	return;
      }
    closure_ = g_cclosure_new (G_CALLBACK (method), user_data, Method::destroy_data);
    g_closure_set_marshal  (closure_,g_cclosure_marshal_generic);
    
    arg_types_ = arg_types;
    
    //counting by value and by pointer args
    num_of_value_args_ = 0;
    num_of_pointer_args_ = 0;
     std::vector<GType>::iterator it = arg_types.begin();
     while ((*it) != G_TYPE_POINTER && it != arg_types.end())
       {
	 num_of_value_args_ ++;
	 ++it;
       }
     num_of_pointer_args_ = arg_types_.size() - num_of_value_args_;
     //g_print ("num value args : %u num pointer arg %u\n", num_of_value_args_, num_of_pointer_args_);
  }


  uint 
  Method::get_num_of_value_args()
  {
    return num_of_value_args_;
  }
  
  uint 
  Method::get_num_of_pointer_args()
  {
    return num_of_pointer_args_;
  }


  bool
  Method::invoke(std::vector<std::string> args, std::vector<void *> pointers)
  {
    if (args.size () + pointers.size() != arg_types_.size())
      {
	g_printerr ("Method::invoke number of arguments does not correspond to the size of argument types\n");
	return false;
      }

    GValue params[arg_types_.size() + pointers.size()];

    for (gulong i=0; i < args.size(); i++)
      {
	params[i] = G_VALUE_INIT;
	
	g_value_init (&params[i],arg_types_[i]);

	if ( !gst_value_deserialize (&params[i],args[i].c_str()))
	  {
	    g_printerr ("Method::invoke string not transformable into gvalue (argument: %s) \n",
			args[i].c_str());
	  }
      }
    
    for (gulong i=args.size(); i < args.size() + pointers.size(); i++)
      {
	params[i] = G_VALUE_INIT;
	
	g_value_init (&params[i], G_TYPE_POINTER);

	g_value_set_pointer (&params[i],pointers[i - args.size()]);

      }
   
    GValue result_value = G_VALUE_INIT;
    gboolean result;
    g_value_init (&result_value, G_TYPE_BOOLEAN);
    
    g_closure_invoke (closure_, &result_value, arg_types_.size(), params, NULL); 
    
    result = g_value_get_boolean (&result_value);

    //unset
    g_value_unset (&result_value);
    for (guint i=0; i < arg_types_.size(); i++)
	g_value_unset (&params[i]);

    return result;
  }

  bool 
  Method::invoke(std::vector<std::string> args)
  {
    std::vector<void *> empty;
    return invoke(args,empty);
  } 
  
  void
  Method::destroy_data (gpointer  data,
			GClosure *closure)
  {
    g_print ("Method::destroy data\n");
  }
  

  void
  Method::set_description (std::string method_name,
			   std::string short_description,
			   std::vector< std::pair<std::string,std::string> > arg_description)
  {
    g_print ("set description\n");
    method_name_ = method_name;
    short_description_ = short_description;
    arg_description_ = arg_description;
  }

  //json formated description
  std::string
  Method::get_description ()
  {
    std::string res;
    
    res.append ("{");
    
    //name
    res.append ("\"function name\": \"");
    res.append (method_name_);
    res.append ("\", ");

    //short description
    res.append ("\"short description\": \"");
    res.append (short_description_);
    res.append ("\", ");

    //arg names and description
    res.append ("\"arguments\": [");
    std::vector<std::pair<std::string,std::string> >::iterator it;
    int j=0;
    for (it = arg_description_.begin() ; it != arg_description_.end(); it++ )
      {
	if (it != arg_description_.begin()) 
	  res.append (", ");
	res.append ("\"name\": \"");
	res.append (it->first);
	res.append ("\", \"short description\": \"");
	res.append (it->second);
	res.append ("\", \"type\": \"");
	res.append (g_type_name (arg_types_[j]));
	res.append ("\"");
	j++;
      }
    res.append ("]");

    res.append ("}");
    return res;
  }
  

}
