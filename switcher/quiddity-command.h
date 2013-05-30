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
      auto_invoke,

      get_signals_description,
      get_signal_description,
      get_signals_description_by_class,
      get_signal_description_by_class,							       
      make_signal_subscriber,										       
      remove_signal_subscriber,
      subscribe_signal,
      unsubscribe_signal,
      list_signal_subscribers,
      list_subscribed_signals,
      list_signal_subscribers_json,
      list_subscribed_signals_json,
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
