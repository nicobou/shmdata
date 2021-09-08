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

#ifndef __SWITCHER_QUIDDITY_DOCUMENTATION_H__
#define __SWITCHER_QUIDDITY_DOCUMENTATION_H__

#include <string>
#include <vector>

namespace switcher {
namespace quiddity {
class Doc {
 public:
  Doc() = default;
  Doc(const std::string& long_name,
      const std::string& class_name,
      const std::string& short_description,
      const std::string& license,
      const std::string& author);
  std::string get_class_name() const;
  std::string get_description() const;
  std::string get_long_name() const;
  std::string get_author() const;
  std::string get_license() const;

 private:
  std::string class_name_{};
  std::string description_{};
  std::string long_name_{};
  std::string author_{};
  std::string license_{};
};
}  // namespace quiddity
}  // namespace switcher
#endif
