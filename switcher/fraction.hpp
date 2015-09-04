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

#ifndef __SWITCHER_FRACTION_H__
#define __SWITCHER_FRACTION_H__

#include <string>
#include <vector>
#include <utility>

namespace switcher {
class Fraction {
 public:
  using ator_t = long long;  // using stoll in from_string static method
  using fraction_t = std::pair<ator_t, ator_t>;
  Fraction() = delete;
  Fraction(ator_t numerator, ator_t denominator);
  fraction_t get() const;
  ator_t numerator() const{return std::get<0>(fraction_);}
  ator_t denominator() const{return std::get<1>(fraction_);}
  static std::pair<bool, Fraction> from_string(const std::string &);
  std::string to_string() const; 

 private:
  fraction_t fraction_;
};

}  // namespace switcher
#endif
