/*
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
 * The Quiddity command class
 */

#include "./quiddity-command.hpp"

namespace switcher {
const std::map<int, const char *>QuiddityCommand::command_names_ = {
  {auto_invoke, "auto_invoke"},
  {create, "create"},
  {create_nick_named, "create_nick_named"},
  {get_class_doc, "get_class_doc"},
  {get_classes, "get_classes"},
  {get_classes_doc, "get_classes_doc"},
  {get_info, "get_info"},
  {get_method_description, "get_method_description"},
  {get_method_description_by_class, "get_method_description_by_class"},
  {get_methods_description, "get_methods_description"},
  {get_methods_description_by_class, "get_methods_description_by_class"},
  {get_properties_description, "get_properties_description"},
  {get_properties_description_by_class,
   "get_properties_description_by_class"},
  {get_property, "get_property"},
  {get_property_description, "get_property_description"},
  {get_property_description_by_class, "get_property_description_by_class"},
  {get_quiddities, "get_quiddities"},
  {get_quiddities_description, "get_quiddities_description"},
  {get_quiddity_description, "get_quiddity_description"},
  {get_signal_description, "get_signal_description"},
  {get_signal_description_by_class, "get_signal_description_by_class"},
  {get_signals_description, "get_signals_description"},
  {get_signals_description_by_class, "get_signals_description_by_class"},
  {has_method, "has_method"},
  {invalid_command, "invalid_command"},
  {invoke, "invoke"},
  {list_property_subscribers, "list_property_subscribers"},
  {list_property_subscribers_json, "list_property_subscribers_json"},
  {list_signal_subscribers, "list_signal_subscribers"},
  {list_signal_subscribers_json, "list_signal_subscribers_json"},
  {list_subscribed_properties, "list_subscribed_properties"},
  {list_subscribed_properties_json, "list_subscribed_properties_json"},
  {list_subscribed_signals, "list_subscribed_signals"},
  {list_subscribed_signals_json, "list_subscribed_signals_json"},
  {make_property_subscriber, "make_property_subscriber"},
  {make_signal_subscriber, "make_signal_subscriber"},
  {quit, "quit"},
  {remove, "remove"},
  {remove_property_subscriber, "remove_property_subscriber"},
  {remove_signal_subscriber, "remove_signal_subscriber"},
  {scan_directory_for_plugins, "scan_directory_for_plugins"},
  {set_property, "set_property"},
  {subscribe_property, "subscribe_property"},
  {subscribe_signal, "subscribe_signal"},
  {unsubscribe_property, "unsubscribe_property"},
  {unsubscribe_signal, "unsubscribe_signal"}
};

void QuiddityCommand::clear() {
  args_.clear();
  vector_arg_.clear();
  result_.clear();
}

void QuiddityCommand::set_id(command id) {
  clear();
  id_ = id;
}

void QuiddityCommand::add_arg(std::string arg) {
  args_.push_back(arg);
}

void QuiddityCommand::set_vector_arg(std::vector<std::string> vector_arg) {
  vector_arg_ = vector_arg;
}

QuiddityCommand::QuiddityCommand():
    json_builder_(std::make_shared<JSONBuilder>()) {
}

JSONBuilder::Node QuiddityCommand::get_json_root_node() {
  json_builder_->reset();
  json_builder_->begin_object();
  json_builder_->add_string_member("command", command_names_.at(id_));
  json_builder_->add_int_member("calling time", (gint) time_);
  json_builder_->set_member_name("arguments");
  json_builder_->begin_array();
  for (auto &it : args_)
    json_builder_->add_string_value(it.c_str());
  json_builder_->end_array();

  json_builder_->set_member_name("vector argument");
  json_builder_->begin_array();
  for (auto &it : vector_arg_)
    json_builder_->add_string_value(it.c_str());
  json_builder_->end_array();
  json_builder_->set_member_name("results");
  json_builder_->begin_array();
  if (result_.empty())
    json_builder_->add_string_value("");
  else
    for (auto &it : result_)
      json_builder_->add_string_value(it.c_str());
  json_builder_->end_array();
  json_builder_->end_object();
  return json_builder_->get_root();
}

QuiddityCommand::command
QuiddityCommand::get_id_from_string(const char *com) {
  std::map < int, const char *>::const_iterator it;
  for (it = command_names_.begin(); it != command_names_.end(); ++it)
    if (g_strcmp0(it->second, com) == 0)
      return (QuiddityCommand::command) it->first;
  return invalid_command;
}

const char *QuiddityCommand::get_string_from_id(QuiddityCommand::command id) {
  std::map < int, const char *>::const_iterator it =
      command_names_.find(id);
  if (it == command_names_.end())
    return command_names_.at(QuiddityCommand::invalid_command);
  return it->second;
}

QuiddityCommand::ptr
QuiddityCommand::parse_command_from_json_reader(JsonReader *reader) {
  int j;
  int num_elements;

  QuiddityCommand::ptr command(new QuiddityCommand());

  // command
  json_reader_read_member(reader, "command");
  command->set_id(QuiddityCommand::get_id_from_string
                  (json_reader_get_string_value(reader)));
  json_reader_end_member(reader);
  // ---

  // invokation time
  json_reader_read_member(reader, "calling time");
  command->time_ = json_reader_get_int_value(reader);
  json_reader_end_member(reader);
  // ---

  // arguments
  json_reader_read_member(reader, "arguments");
  num_elements = json_reader_count_elements(reader);
  for (j = 0; j < num_elements; j++) {
    json_reader_read_element(reader, j);
    const gchar *str = json_reader_get_string_value(reader);
    if (nullptr != str)
      command->add_arg(str);
    else
      command->add_arg("null");
    json_reader_end_element(reader);
  }
  json_reader_end_member(reader);
  // ---

  // vector arguments
  json_reader_read_member(reader, "vector argument");
  num_elements = json_reader_count_elements(reader);
  std::vector<std::string> string_vect_arg;
  for (j = 0; j < num_elements; j++) {
    json_reader_read_element(reader, j);
    const char *stringValue = json_reader_get_string_value(reader);
    if (stringValue == nullptr)
      stringValue = "null";
    string_vect_arg.push_back(stringValue);
    json_reader_end_element(reader);
  }
  json_reader_end_member(reader);
  command->set_vector_arg(string_vect_arg);
  // ---

  // results
  json_reader_read_member(reader, "results");
  num_elements = json_reader_count_elements(reader);
  std::vector<std::string> expected_result;
  for (j = 0; j < num_elements; j++) {
    json_reader_read_element(reader, j);
    const char *string_value = json_reader_get_string_value(reader);
    if (nullptr != string_value)
      expected_result.push_back(string_value);
    json_reader_end_element(reader);
  }
  json_reader_end_member(reader);
  command->expected_result_ = expected_result;
  // ---

  json_reader_end_element(reader);
  return command;
}
}
