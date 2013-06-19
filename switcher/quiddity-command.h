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
#include <map>
#include <memory>
#include "json-builder.h"

namespace switcher
{
 
  struct QuiddityCommand
  {
    typedef std::shared_ptr<QuiddityCommand> ptr;
    enum command 
    {
      invalid_command = -1,
      auto_invoke = 0,
      create, 
      create_nick_named,
      get_class_doc, 
      get_classes,  
      get_classes_doc,
      get_method_description,
      get_method_description_by_class,
      get_methods_description,
      get_methods_description_by_class,
      get_properties_description,
      get_properties_description_by_class,
      get_property,
      get_property_description,
      get_property_description_by_class,
      get_quiddities, 
      get_quiddities_description, 
      get_quiddity_description,
      get_signal_description,
      get_signal_description_by_class,							       
      get_signals_description,
      get_signals_description_by_class,
      has_method,
      invoke,
      list_property_subscribers,
      list_property_subscribers_json,
      list_signal_subscribers,
      list_signal_subscribers_json,
      list_subscribed_properties,
      list_subscribed_properties_json,
      list_subscribed_signals,
      list_subscribed_signals_json,
      make_property_subscriber,
      make_signal_subscriber,										       
      remove,
      remove_property_subscriber,
      remove_signal_subscriber,
      set_property,
      subscribe_property,
      subscribe_signal,
      unsubscribe_property,
      unsubscribe_signal
    };

    QuiddityCommand ();
    command name_;//FIXME refactor into id_
    std::vector<std::string> args_;
    std::vector<std::string> vector_arg_;
    std::vector<std::string> result_;
    std::vector<std::string> expected_result_;
    gint64 time_;////monotonic time, in microseconds 
    void clear();
    void set_name (command id); //FIXME refactor into set_id
    void add_arg (std::string arg);
    void set_vector_arg (std::vector<std::string> vector_arg);
    static command get_id_from_string (const char *com);
    static const char *get_string_from_id (QuiddityCommand::command id);
    static QuiddityCommand::ptr parse_command_from_json_reader (JsonReader *reader);
    JSONBuilder::Node get_json_root_node ();
    JSONBuilder::ptr json_builder_;
    static const std::map<int, const char *> command_names_;
  };
  
} // end of namespace

#endif // ifndef
