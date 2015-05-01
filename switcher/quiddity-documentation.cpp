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

#include "./quiddity-documentation.hpp"

namespace switcher {
QuiddityDocumentation::QuiddityDocumentation(const std::string &long_name,
                                             const std::string &category,
                                             const std::string &short_description,
                                             const std::string &license,
                                             const std::string &class_name,
                                             const std::string &author) :
    category_(std::move(category)),
    class_name_(std::move(class_name)),
    description_(std::move(short_description)),
    long_name_(std::move(long_name)),
  author_(std::move(author)),
  license_(std::move(license)),
  json_description_(std::make_shared<JSONBuilder>()) {
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
  json_description_->add_string_member("long name", long_name_.c_str());
  json_description_->add_string_member("category", category_.c_str());
  json_description_->add_string_member("short description",
                                       description_.c_str());
  json_description_->add_string_member("license", license_.c_str());
  json_description_->add_string_member("class name", class_name_.c_str());
  json_description_->add_string_member("author", author_.c_str());
  json_description_->end_object();
}

JSONBuilder::Node QuiddityDocumentation::get_json_root_node() {
  make_json_description();
  return json_description_->get_root();
}

}  // namespace switcher
