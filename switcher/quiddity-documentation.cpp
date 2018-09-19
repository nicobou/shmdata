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
#include <iostream>
#include <sstream>

namespace switcher {
quid::Doc::Doc(const std::string& long_name,
               const std::string& class_name,
               const std::string& category,
               const std::string& tags,
               const std::string& short_description,
               const std::string& license,
               const std::string& author)
    : category_(category),
      class_name_(class_name),
      description_(short_description),
      long_name_(long_name),
      author_(author),
      license_(license) {
  // parsing tags since vector initialization like {"writer", "reader"} does
  // not pass MACRO arguments:
  std::istringstream ss(tags);  // Turn the string into a stream
  std::string tok;
  while (std::getline(ss, tok, '/')) tags_.push_back(tok);
}

std::string quid::Doc::get_category() const { return category_; }

std::string quid::Doc::get_class_name() const { return class_name_; }

std::string quid::Doc::get_description() const { return description_; }

std::string quid::Doc::get_long_name() const { return long_name_; }

std::string quid::Doc::get_author() const { return author_; }

std::string quid::Doc::get_license() const { return license_; }

std::vector<std::string> quid::Doc::get_tags() const { return tags_; };

}  // namespace switcher
