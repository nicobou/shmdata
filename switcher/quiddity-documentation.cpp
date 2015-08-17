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

#include <iostream>
#include <sstream>
#include "./quiddity-documentation.hpp"

namespace switcher {
QuiddityDocumentation::QuiddityDocumentation(const std::string &long_name,
                                             const std::string &class_name,
                                             const std::string &category,
                                             const std::string &tags,
                                             const std::string &short_description,
                                             const std::string &license,
                                             const std::string &author) :
    category_(category),
  class_name_(class_name),
  description_(short_description),
  long_name_(long_name),
  author_(author),
  license_(license),
  json_description_(std::make_shared<JSONBuilder>()) {
  // parsing tags since vector initialization like {"writer", "reader"} does
  // not pass MACRO arguments:
  std::istringstream ss(tags); // Turn the string into a stream
  std::string tok;  
  while(std::getline(ss, tok, '/'))
    tags_.push_back(tok);
}

std::string QuiddityDocumentation::get_category() const {
  return category_;
}

std::string QuiddityDocumentation::get_class_name() const {
  return class_name_;
}

std::string QuiddityDocumentation::get_description() const {
  return description_;
}

std::string QuiddityDocumentation::get_long_name() const {
  return long_name_;
}

std::string QuiddityDocumentation::get_author() const {
  return author_;
}

std::string QuiddityDocumentation::get_license() const {
  return license_;
}

void QuiddityDocumentation::make_json_description() {
  json_description_ = std::make_shared<JSONBuilder>();
  json_description_->reset();
  json_description_->begin_object();
  json_description_->add_string_member("class", class_name_.c_str());
  json_description_->add_string_member("name", long_name_.c_str());
  json_description_->add_string_member("category", category_.c_str());
  json_description_->set_member_name("tags");
  json_description_->begin_array();
  for (auto &it: tags_)
    json_description_->add_string_value(it.c_str());
  json_description_->end_array();
  json_description_->add_string_member("description",
                                       description_.c_str());
  json_description_->add_string_member("license", license_.c_str());
  json_description_->add_string_member("author", author_.c_str());
  json_description_->end_object();
}

JSONBuilder::Node QuiddityDocumentation::get_json_root_node() {
  make_json_description();
  return json_description_->get_root();
}

}  // namespace switcher
