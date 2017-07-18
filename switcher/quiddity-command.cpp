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
const std::map<int, const char*> QuiddityCommand::command_names_ = {
    {auto_invoke, "auto_invoke"},
    {create, "create"},
    {create_nick_named, "create_nick_named"},
    {get_class_doc, "get_class_doc"},
    {get_classes, "get_classes"},
    {get_classes_doc, "get_classes_doc"},
    {get_method_description, "get_method_description"},
    {get_method_description_by_class, "get_method_description_by_class"},
    {get_methods_description, "get_methods_description"},
    {get_methods_description_by_class, "get_methods_description_by_class"},
    {get_quiddities_description, "get_quiddities_description"},
    {get_quiddity_description, "get_quiddity_description"},
    {get_signal_description, "get_signal_description"},
    {get_signal_description_by_class, "get_signal_description_by_class"},
    {get_signals_description, "get_signals_description"},
    {get_signals_description_by_class, "get_signals_description_by_class"},
    {has_method, "has_method"},
    {invalid_command, "invalid_command"},
    {invoke, "invoke"},
    {list_signal_subscribers, "list_signal_subscribers"},
    {list_subscribed_signals, "list_subscribed_signals"},
    {list_subscribed_signals_json, "list_subscribed_signals_json"},
    {make_signal_subscriber, "make_signal_subscriber"},
    {quit, "quit"},
    {remove, "remove"},
    {remove_signal_subscriber, "remove_signal_subscriber"},
    {set_property, "set_property"},
    {subscribe_property, "subscribe_property"},
    {subscribe_signal, "subscribe_signal"},
    {unsubscribe_property, "unsubscribe_property"},
    {unsubscribe_signal, "unsubscribe_signal"}};

void QuiddityCommand::clear() {
  args_.clear();
  vector_arg_.clear();
  result_.clear();
}

void QuiddityCommand::set_id(command id) {
  clear();
  id_ = id;
}

void QuiddityCommand::add_arg(std::string arg) { args_.push_back(arg); }

void QuiddityCommand::set_vector_arg(std::vector<std::string> vector_arg) {
  vector_arg_ = vector_arg;
}

InfoTree::ptr QuiddityCommand::get_info_tree() const {
  auto tree = InfoTree::make();
  tree->graft("command", InfoTree::make(command_names_.at(id_)));
  for (unsigned int i = 0; i < args_.size(); i++) {
    tree->graft(std::string("arguments.") + std::to_string(i), InfoTree::make(args_[i]));
  }
  tree->tag_as_array("arguments", true);
  if (vector_arg_.empty()) {
    tree->graft("vector argument.", InfoTree::make());
  } else {
    for (unsigned int i = 0; i < vector_arg_.size(); i++) {
      tree->graft(std::string("vector argument.") + std::to_string(i),
                  InfoTree::make(vector_arg_[i]));
    }
  }
  tree->tag_as_array("vector argument", true);
  if (result_.empty()) {
    tree->graft("results.0.", InfoTree::make(""));
  } else {
    for (unsigned int i = 0; i < result_.size(); i++) {
      tree->graft(std::string("results.") + std::to_string(i), InfoTree::make(result_[i]));
    }
  }
  tree->tag_as_array("results", true);
  return tree;
}

QuiddityCommand::command QuiddityCommand::get_id_from_string(const char* com) {
  std::map<int, const char*>::const_iterator it;
  for (it = command_names_.begin(); it != command_names_.end(); ++it)
    if (g_strcmp0(it->second, com) == 0) return (QuiddityCommand::command)it->first;
  return invalid_command;
}

const char* QuiddityCommand::get_string_from_id(QuiddityCommand::command id) {
  std::map<int, const char*>::const_iterator it = command_names_.find(id);
  if (it == command_names_.end()) return command_names_.at(QuiddityCommand::invalid_command);
  return it->second;
}

QuiddityCommand::ptr QuiddityCommand::make_command_from_tree(InfoTree::ptr tree) {
  QuiddityCommand::ptr command = std::make_shared<QuiddityCommand>();
  std::string command_str = tree->branch_get_value("command");
  command->set_id(QuiddityCommand::get_id_from_string(command_str.c_str()));
  {
    std::string args_str("arguments.");
    auto arguments = tree->get_child_keys(args_str);
    for (auto& it : arguments) {
      std::string arg = tree->branch_get_value(args_str + it);
      if (!arg.empty()) {
        command->add_arg(arg);
      } else {
        command->add_arg("null");
      }
    }
  }
  {
    std::string vect_str("vector argument.");
    auto arguments = tree->get_child_keys(vect_str);
    std::vector<std::string> string_vect_arg;
    for (auto& it : arguments) {
      std::string arg = tree->branch_get_value(vect_str + it);
      if (!arg.empty()) {
        string_vect_arg.push_back(arg);
      } else {
        string_vect_arg.push_back("null");
      }
    }
    command->set_vector_arg(string_vect_arg);
  }
  {
    std::string res_str("results.");
    auto results = tree->get_child_keys(res_str);
    std::vector<std::string> expected_result;
    for (auto& it : results) {
      std::string res = tree->branch_get_value(res_str + it);
      if (!res.empty()) expected_result.push_back(res);
    }
    command->expected_result_ = expected_result;
  }
  return command;
}
}
