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

#include <gst/gst.h>
#include <sstream>
#include <algorithm>
#include "./string-utils.hpp"

namespace switcher {

std::string StringUtils::replace(const std::string &orig,
                                 const char to_replace,
                                 const std::string &replacement){
  std::istringstream ss(orig); // Turn the string into a stream
  std::string tok;
  std::getline(ss, tok, to_replace);
  std::string res = tok;
  while(std::getline(ss, tok, ' '))
    res += replacement + tok;
  return res;
}

std::string StringUtils::replace_chars(const std::string &orig,
                                       const std::vector<char> &to_replace,
                                       const char replacement){
  std::string res = orig;
  // FIXME iterate only once
  for (auto &it: to_replace) 
    std::replace( res.begin(), res.end(), it, replacement);
  return res;
}

}  // namespace switcher
