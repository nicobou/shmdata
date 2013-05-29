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
 * The Quiddity command
 */


#ifndef __SWITCHER_QUIDDITY_COMMAND_H__
#define __SWITCHER_QUIDDITY_COMMAND_H__

#include <string>
#include <vector>
#include <memory>

namespace switcher
{
 
  class QuiddityCommand
  {
  public:
    typedef std::shared_ptr<QuiddityCommand> ptr;
    
    enum command 
    {
      get_classes,  
      get_quiddities, 
      get_classes_doc,
      get_class_doc, 
      get_quiddity_description,
      get_quiddities_description, 

      create, 
      create_nick_named,
      remove,


      get_properties_description,
      get_property_description,
      get_properties_description_by_class,
      get_property_description_by_class,
      set_property,
      get_property,

      make_property_subscriber,
      remove_property_subscriber,
      subscribe_property,
      unsubscribe_property,
      list_property_subscribers,
      list_subscribed_properties,
      list_property_subscribers_json,
      list_subscribed_properties_json,

      get_methods_description,
      get_method_description,
      get_methods_description_by_class,
      get_method_description_by_class,
      invoke,
     /* bool auto_invoke  (std::string method_name, */
     /* 			 std::vector<std::string> args);   */

      /* //doc (json formatted) */
      /* std::string get_signals_description (std::string quiddity_name); */
      /* std::string get_signal_description (std::string quiddity_name,  */
      /* 					  std::string signal_name); */
      /* //following "by_class" methods provide properties available after creation only,  */
      /* //avoiding possible properties created dynamically */
      /* std::string get_signals_description_by_class (std::string class_name);  */
      /* std::string get_signal_description_by_class (std::string class_name,  */
      /* 						   std::string signal_name);  */
    
      /* bool make_signal_subscriber (std::string subscriber_name, */
      /* 				   void (*callback)(std::string subscriber_name, */
      /* 						    std::string quiddity_name, */
      /* 						    std::string signal_name, */
      /* 						    std::vector<std::string> params, */
      /* 						    void *user_data), */
      /* 				   void *user_data); */
      /* bool remove_signal_subscriber (std::string subscriber_name); */
      /* bool subscribe_signal (std::string subscriber_name, */
      /* 			     std::string quiddity_name, */
      /* 			     std::string signal_name); */
      /* bool unsubscribe_signal (std::string subscriber_name, */
      /* 			       std::string quiddity_name, */
      /* 			       std::string signal_name); */

      
      /* std::vector<std::string>  */
      /* 	list_signal_subscribers (); */
      /* std::vector<std::pair<std::string, std::string> >  */
      /* 	list_subscribed_signals (std::string subscriber_name); */
      /* //json //FIXME implement or remove */
      /* std::string  */
      /* 	list_signal_subscribers_json (); */
      /* std::string  */
      /* 	list_subscribed_signals_json (std::string subscriber_name); */
   
    };
    command name_;
    std::vector<std::string> args_;
    std::vector<std::string> vector_arg_;
    std::vector<std::string> result_;
 
    void clear();
    void set_name (command name);
    void add_arg (std::string arg);
    void set_vector_arg (std::vector<std::string> vector_arg);
  };
  
} // end of namespace

#endif // ifndef
