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

#include "quiddity-documentation.h"

namespace switcher
{
  QuiddityDocumentation::QuiddityDocumentation (std::string category, 
						std::string class_name, 
						std::string description)
  {
    category_ = category;
    class_name_ = class_name;
    description_ = description;
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

  void 
  QuiddityDocumentation::make_json_description ()
  {
    json_description_.reset (new JSONBuilder());
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("category",category_.c_str ());
    json_description_->add_string_member ("class name",class_name_.c_str ());
    json_description_->add_string_member ("short description",description_.c_str ());
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
}
