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

#include "./doc.hpp"
#include <iostream>
#include <sstream>

namespace switcher {
namespace quiddity {
quiddity::Doc::Doc(const std::string& long_name,
                   const std::string& kind,
                   const std::string& short_description,
                   const std::string& license,
                   const std::string& author)
    : kind_(kind),
      description_(short_description),
      long_name_(long_name),
      author_(author),
      license_(license) {}

std::string quiddity::Doc::get_kind() const { return kind_; }

std::string quiddity::Doc::get_description() const { return description_; }

std::string quiddity::Doc::get_long_name() const { return long_name_; }

std::string quiddity::Doc::get_author() const { return author_; }

std::string quiddity::Doc::get_license() const { return license_; }

}  // namespace quiddity
}  // namespace switcher
