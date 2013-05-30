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
 * The Quiddity command class
 */

#include "quiddity-command.h"

namespace switcher
{

  const std::map<int, const char *> QuiddityCommand::command_names_ = {
    {auto_invoke,"auto_invoke"},
    {create, "create"},
    {create_nick_named,"create_nick_named"},
    {get_class_doc, "get_class_doc"},
    {get_classes,  "get_classes"},
    {get_classes_doc,"get_classes_doc"},
    {get_method_description,"get_method_description"},
    {get_method_description_by_class,"get_method_description_by_class"},
    {get_methods_description,"get_methods_description"},
    {get_methods_description_by_class,"get_methods_description_by_class"},
    {get_properties_description,"get_properties_description"},
    {get_properties_description_by_class,"get_properties_description_by_class"},
    {get_property,"get_property"},
    {get_property_description,"get_property_description"},
    {get_property_description_by_class,"get_property_description_by_class"},
    {get_quiddities, "get_quiddities"},
    {get_quiddities_description, "get_quiddities_description"},
    {get_quiddity_description,"get_quiddity_description"},
    {get_signal_description,"get_signal_description"},
    {get_signal_description_by_class,	"get_signal_description_by_class"},		       
    {get_signals_description,"get_signals_description"},
    {get_signals_description_by_class,"get_signals_description_by_class"},
    {has_method,"has_method"},
    {invoke,"invoke"},
    {list_property_subscribers,"list_property_subscribers"},
    {list_property_subscribers_json,"list_property_subscribers_json"},
    {list_signal_subscribers,"list_signal_subscribers"},
    {list_signal_subscribers_json,"list_signal_subscribers_json"},
    {list_subscribed_properties,"list_subscribed_properties"},
    {list_subscribed_properties_json,"list_subscribed_properties_json"},
    {list_subscribed_signals,"list_subscribed_signals"},
    {list_subscribed_signals_json,"list_subscribed_signals_json"},
    {make_property_subscriber,"make_property_subscriber"},
    {make_signal_subscriber, "make_signal_subscriber"},
    {remove,"remove"},
    {remove_property_subscriber,"remove_property_subscriber"},
    {remove_signal_subscriber,"remove_signal_subscriber"},
    {set_property,"set_property"},
    {subscribe_property,"subscribe_property"},
    {subscribe_signal,"subscribe_signal"},
    {unsubscribe_property,"unsubscribe_property"},
    {unsubscribe_signal,"unsubscribe_signal"}};

  void
  QuiddityCommand::clear()
  {
    args_.clear ();
    vector_arg_.clear ();
    result_.clear ();
  }

  void 
  QuiddityCommand::set_name (command name)
  {
    clear ();
    name_ = name;
  } 
  
  void 
  QuiddityCommand::add_arg (std::string arg)
  {
    args_.push_back (arg);
  } 
 
  void 
  QuiddityCommand::set_vector_arg (std::vector<std::string> vector_arg)
  {
    vector_arg_ = vector_arg;
  } 

  QuiddityCommand::QuiddityCommand ()
  {
    json_builder_.reset (new JSONBuilder ());
  }
  
  JSONBuilder::Node
  QuiddityCommand::get_json_root_node ()
  {
    json_builder_->reset ();
    json_builder_->begin_object ();
    json_builder_->add_string_member ("command", command_names_.at (name_));
    json_builder_->set_member_name ("arguments");
    json_builder_->begin_array ();    
    for (auto& it: args_) 
      {
	json_builder_->begin_object ();
      	json_builder_->add_string_member ("value", it.c_str ());
	json_builder_->end_object ();
      }
    json_builder_->end_array ();    
    
    json_builder_->set_member_name ("vector argument");
    json_builder_->begin_array ();
    for (auto& it: vector_arg_)
      {
	json_builder_->begin_object ();
      	json_builder_->add_string_member ("value", it.c_str ());
	json_builder_->end_object ();
      }
    json_builder_->end_array ();

    json_builder_->set_member_name ("results");
    json_builder_->begin_array ();
    for (auto& it: result_)
      {
      	json_builder_->begin_object ();
	json_builder_->add_string_member ("value", it.c_str ());
	json_builder_->end_object ();
      }
    json_builder_->end_array ();
    
    json_builder_->end_object ();
    return json_builder_->get_root ();
  }
}
