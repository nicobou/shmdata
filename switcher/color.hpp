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

#ifndef __SWITCHER_COLOR_H__
#define __SWITCHER_COLOR_H__

#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace switcher {
class Color {
 public:
  using channel_t = uint8_t;
  struct color_t {
    color_t(channel_t r, channel_t g, channel_t b, channel_t a);
    channel_t r{1};
    channel_t g{1};
    channel_t b{1};
    channel_t a{1};
  };
  Color() = delete;
  Color(channel_t r, channel_t g, channel_t b, channel_t a);
  color_t get() const;
  channel_t red() const;
  channel_t green() const;
  channel_t blue() const;
  channel_t alpha() const;
  std::string to_string() const;
  static std::pair<bool, Color> from_string(const std::string&);

  friend std::ostream& operator<<(std::ostream& out, const Color& color) {
    out << color.to_string();
    return out;
  }

 private:
  color_t color_;
};

}  // namespace switcher
#endif
