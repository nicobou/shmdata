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

#include "quiddity-documentation.h"

namespace switcher
{
  QuiddityDocumentation::QuiddityDocumentation (std::string long_name,
						std::string category, 
						std::string short_description,
						std::string license,
						std::string class_name, 
						std::string author)
  {
    category_ = category;
    class_name_ = class_name;
    description_ = short_description;
    long_name_ = long_name;
    author_ = author;
    license_ = license;
  }

  std::string 
  QuiddityDocumentation::get_category () const
  {
    return category_;
  }
 
  std::string 
  QuiddityDocumentation::get_class_name () const
  {
    return class_name_;
  }
  
  std::string
  QuiddityDocumentation::get_description () const
  {
    return description_;
  }

  std::string
  QuiddityDocumentation::get_long_name () const
  {
    return long_name_;
  }

  std::string
  QuiddityDocumentation::get_author () const
  {
    return author_;
  }

  std::string
  QuiddityDocumentation::get_license () const
  {
    return license_;
  }


  void 
  QuiddityDocumentation::make_json_description ()
  {
    json_description_.reset (new JSONBuilder());
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("long name",long_name_.c_str ());
    json_description_->add_string_member ("category",category_.c_str ());
    json_description_->add_string_member ("short description",description_.c_str ());
    json_description_->add_string_member ("license",license_.c_str ());
    json_description_->add_string_member ("class name",class_name_.c_str ());
    json_description_->add_string_member ("author",author_.c_str ());
    json_description_->end_object ();
  }
  
  std::string 
  QuiddityDocumentation::get_json_documentation () 
  {
    make_json_description ();
    return json_description_->get_string (true);;
  }

  JSONBuilder::Node 
  QuiddityDocumentation::get_json_root_node ()
  {
    make_json_description ();
    return json_description_->get_root ();
  }

  void
  QuiddityDocumentation::set_category (std::string category)
  {
    category_ = category;
  }

  void
  QuiddityDocumentation::set_class_name (std::string class_name)
  {
    class_name_ = class_name;
  }

  void
  QuiddityDocumentation::set_description (std::string description)
  {
    description_ = description;
  }

  void
  QuiddityDocumentation::set_long_name (std::string long_name)
  {
    long_name_ = long_name;
  }

  void
  QuiddityDocumentation::set_author (std::string author)
  {
    author_ = author;
  }

  void
  QuiddityDocumentation::set_license (std::string license)
  {
    license_ = license;
  }
}
