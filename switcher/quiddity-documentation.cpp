/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/quiddity-documentation.h"

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
  
  std::string 
  QuiddityDocumentation::get_json_documentation () const
  {
    std::string documentation;
    documentation.append("{");

    documentation.append("\"category\":\"");
    documentation.append(category_);
    documentation.append("\",");

    documentation.append("\"class name\":\"");
    documentation.append(class_name_);
    documentation.append("\",");

    documentation.append("\"description\":\"");
    documentation.append(description_);
    documentation.append("\"");

    documentation.append("}");
    return documentation;
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
