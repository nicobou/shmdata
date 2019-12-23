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

#include "./fraction.hpp"
#include <cctype>
#include <utility>

namespace switcher {

Fraction::Fraction(ator_t num, ator_t denom) : num_(num), denom_(denom) {}

Fraction::fraction_t Fraction::get() const { return std::make_pair(num_, denom_); }

std::pair<bool, Fraction> Fraction::from_string(const std::string& str) {
  if (!isdigit(*str.begin()) &&
      !('-' == *str.begin() && str.begin() + 1 != str.end() && isdigit(*(str.begin() + 1)))) {
#ifdef DEBUG
    std::cerr << str << " cannot be parsed as a fraction" << '\n';
#endif
    return std::make_pair(false, Fraction(0, 0));
  }
  size_t pos;
  long long num = std::stoll(str, &pos, 0);
  if (std::string::npos == pos || pos + 1 >= str.size() || !isdigit(*(str.begin() + pos + 1))) {
#ifdef DEBUG
    std::cerr << str << " cannot be parsed as a fraction" << '\n';
#endif
    return std::make_pair(false, Fraction(0, 0));
  }
  long long denom = std::stoll(str.substr(pos + 1, std::string::npos), &pos, 0);
  return std::make_pair(true, Fraction(num, denom));
}

std::string Fraction::to_string() const {
  return std::to_string(numerator()) + "/" + std::to_string(denominator());
}

}  // namespace switcher
