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
#include <glib.h>
#include <algorithm>
#include <cctype>
#include <sstream>
#include "./scope-exit.hpp"

namespace switcher {

std::string stringutils::replace_char(const std::string& orig,
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

std::string stringutils::replace_string(const std::string& orig,
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

void stringutils::toupper(std::string& str) {
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
}
void stringutils::tolower(std::string& str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
}

bool stringutils::starts_with(const std::string& str, const std::string& suffix) {
  auto lower_str = std::string(str);
  auto lower_suffix = std::string(suffix);
  stringutils::tolower(lower_str);
  stringutils::tolower(lower_suffix);
  return str.size() >= suffix.size() && lower_str.find(lower_suffix) == 0;
}

bool stringutils::ends_with(const std::string& str, const std::string& suffix) {
  auto str_len = str.size();
  auto suffix_len = suffix.size();
  auto lower_str = std::string(str);
  auto lower_suffix = std::string(suffix);
  stringutils::tolower(lower_str);
  stringutils::tolower(lower_suffix);
  return str_len >= suffix_len &&
         lower_str.find(lower_suffix, str_len - suffix_len) != std::string::npos;
}

std::string stringutils::base64_encode(const std::string& str) {
  return std::string(g_base64_encode(reinterpret_cast<const guchar*>(str.c_str()), str.size()));
}

std::string stringutils::base64_decode(const std::string& str) {
  gsize str_size = 2048;
  guchar* decoded_guchar = g_base64_decode(str.c_str(), &str_size);
  On_scope_exit {
    if (decoded_guchar) g_free(decoded_guchar);
  };
  gchar* decoded_char = g_strdup_printf("%s", decoded_guchar);
  On_scope_exit {
    if (decoded_char) g_free(decoded_char);
  };
  if (!decoded_char) return std::string();
  return std::string(decoded_char);
}

std::string stringutils::escape_json(const std::string& str) {
  std::ostringstream o;
  for (auto c = str.cbegin(); c != str.cend(); c++) {
    switch (*c) {
      case '"':
        o << "\\\"";
        break;
      case '\\':
        o << "\\\\";
        break;
      case '\b':
        o << "\\b";
        break;
      case '\f':
        o << "\\f";
        break;
      case '\n':
        o << "\\n";
        break;
      case '\r':
        o << "\\r";
        break;
      case '\t':
        o << "\\t";
        break;
      default:
        if ('\x00' <= *c && *c <= '\x1f') {
          o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (int)*c;
        } else {
          o << *c;
        }
    }
  }
  return o.str();
}

}  // namespace switcher
