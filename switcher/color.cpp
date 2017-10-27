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

#include "./color.hpp"
#include <glib.h>
#include <cctype>
#include <utility>

namespace switcher {

Color::Color(channel_t r, channel_t g, channel_t b, channel_t a) : color_(r, g, b, a) {}

Color::color_t::color_t(channel_t red, channel_t green, channel_t blue, channel_t alpha)
    : r(red), g(green), b(blue), a(alpha) {}

Color::color_t Color::get() const { return color_; }

std::pair<bool, Color> Color::from_string(const std::string& str) {
  if (str.size() != 8 || !isxdigit(str[0]) || !isxdigit(str[1]) || !isxdigit(str[2]) ||
      !isxdigit(str[3]) || !isxdigit(str[4]) || !isxdigit(str[5]) || !isxdigit(str[6]) ||
      !isxdigit(str[7])) {
#ifdef DEBUG
    std::cerr << str << " cannot be parsed as a color" << '\n';
#endif
    return std::make_pair(false, Color(0, 0, 0, 0));
  }
  auto val = strtoul(str.c_str(), NULL, 16);
  channel_t r = val >> 24 & 0xFF;
  channel_t g = val >> 16 & 0xFF;
  channel_t b = val >> 8 & 0xFF;
  channel_t a = val & 0xFF;

  return std::make_pair(true, Color(r, g, b, a));
}

std::string Color::to_string() const {
  char res[9];
  res[8] = '\0';
  sprintf(res, "%02X%02X%02X%02X", color_.r, color_.g, color_.b, color_.a);
  return std::string(res);
}

Color::channel_t Color::red() const { return color_.r; }

Color::channel_t Color::green() const { return color_.g; }

Color::channel_t Color::blue() const { return color_.b; }

Color::channel_t Color::alpha() const { return color_.a; }

}  // namespace switcher
