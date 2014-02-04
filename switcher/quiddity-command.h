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
 
  class QuiddityCommand
  {
  public:
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
      quit,
      remove,
      remove_property_subscriber,
      remove_signal_subscriber,
      rename,
      scan_directory_for_plugins,
      set_property,
      subscribe_property,
      subscribe_signal,
      unsubscribe_property,
      unsubscribe_signal
    };

    QuiddityCommand ();
    command id_;
    std::vector<std::string> args_;
    std::vector<std::string> vector_arg_;
    std::vector<std::string> result_;
    std::vector<std::string> expected_result_;
    bool success_;
    gint64 time_;////monotonic time, in microseconds 
    void clear();
    void set_id (command id); 
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
