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

#ifndef __SWITCHER_QUIDDITY_DOCUMENTATION_H__
#define __SWITCHER_QUIDDITY_DOCUMENTATION_H__

#include <string>
#include "json-builder.h"

namespace switcher
{

  class QuiddityDocumentation 
  {

  public:
    QuiddityDocumentation (std::string long_name,
			   std::string category, 
			   std::string short_description,
			   std::string license,
			   std::string class_name, 
			   std::string author);
    
    std::string get_category () const;
    std::string get_class_name () const;
    std::string get_description () const;
    std::string get_long_name () const;
    std::string get_author () const;
    std::string get_license () const;

    void set_category (std::string category);
    void set_class_name (std::string class_name);
    void set_description (std::string description);
    void set_long_name (std::string long_name);
    void set_author (std::string author);
    void set_license (std::string license);


    std::string get_json_documentation ();
    JSONBuilder::Node get_json_root_node ();
    
  private:
    std::string category_;
    std::string class_name_;
    std::string description_;
    std::string long_name_;
    std::string author_;
    std::string license_;

    JSONBuilder::ptr json_description_;
    void make_json_description ();
  };

} // end of namespace

#endif // ifndef

