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

#include "./string-utils.hpp"
#include <gst/gst.h>
#include <algorithm>
#include <sstream>

namespace switcher {

std::string StringUtils::replace_char(const std::string& orig,
                                      const char to_replace,
                                      const std::string& replacement) {
  std::string escaped = std::string();
  std::size_t i = 0;
  while (std::string::npos != i) {
    auto found = orig.find(to_replace, i);
    if (i != found) escaped += std::string(orig, i, found - i);
    if (std::string::npos != found) {
      escaped += replacement;
      i = ++found;
    } else {
      i = std::string::npos;
    }
  }
  return escaped;
}

std::string StringUtils::replace_string(const std::string& orig,
                                        const std::string& to_replace,
                                        const std::string& replacement) {
  std::string unescaped = std::string();
  std::size_t i = 0;
  while (std::string::npos != i) {
    std::size_t found = orig.find(to_replace, i);
    if (i != found) unescaped += std::string(orig, i, found - i);
    if (std::string::npos != found) {
      unescaped += replacement;
      i = found + to_replace.size();
    } else {
      i = std::string::npos;
    }
  }
  return unescaped;
}

}  // namespace switcher
