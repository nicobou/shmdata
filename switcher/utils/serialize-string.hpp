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

#ifndef __SWITCHER_CONVERT_WITH_STRING_H__
#define __SWITCHER_CONVERT_WITH_STRING_H__

#include <cctype>  // tolower
#include <iostream>
#include <sstream>  // fallback to operator << for serialize
#include <string>
#include <utility>
#include "./is-specialization-of.hpp"
#include "./string-utils.hpp"
#include "./templated-sequence.hpp"  // tuple serialization

namespace switcher {
namespace serialize {
// arithmetic
template <typename V,
          typename W = V,
          typename std::enable_if<
              std::is_same<V, int>::value || std::is_same<V, short>::value ||
              std::is_same<V, long>::value || std::is_same<V, long long>::value ||
              std::is_same<V, unsigned int>::value || std::is_same<V, unsigned short>::value ||
              std::is_same<V, unsigned long>::value || std::is_same<V, unsigned long long>::value ||
              std::is_same<V, float>::value || std::is_same<V, double>::value ||
              std::is_same<V, long double>::value>::type* = nullptr>
std::string apply(const W& val) {
  return std::to_string(val);
}

// boolean
template <typename V,
          typename W = V,
          typename std::enable_if<std::is_same<V, bool>::value>::type* = nullptr>
std::string apply(const W& val) {
  if (val) return std::string("true");
  return std::string("false");
}

// string
template <typename V,
          typename W = V,
          typename std::enable_if<std::is_same<V, std::string>::value>::type* = nullptr>
std::string apply(const W& val) {
  return std::string(val);
}

// char
template <typename V,
          typename W = V,
          typename std::enable_if<std::is_same<V, char>::value>::type* = nullptr>
std::string apply(const W& val) {
  return std::string(1, val);
}

template <
    typename V,
    typename W = V,
    typename std::enable_if<std::is_same<V, wchar_t>::value || std::is_same<V, char32_t>::value ||
                            std::is_same<V, char16_t>::value>::type* = nullptr>
std::string apply(const W&) {
  static_assert(true, "wchar_t, char16_t and char32_t not supported by serialize-string.hpp");
  return std::string();
}

// tuple
static const std::string tuple_comma_esc_string = "__comma__";

std::string esc_for_tuple(const std::string& str);

void append_targs(std::string*, size_t);
template <typename F, typename... U>
void append_targs(std::string* res, size_t pos, F first, U... args) {
  if (0 == pos)
    res->append(StringUtils::replace_char(apply<F>(first), ',', tuple_comma_esc_string));
  else
    res->append(std::string(",") +
                StringUtils::replace_char(apply<F>(first), ',', tuple_comma_esc_string));
  append_targs(res, pos + 1, args...);
}
template <typename... U, int... S>
void append_tuple_call(std::string* res, const std::tuple<U...>& tup, tseq<S...>) {
  append_targs(res, 0, std::get<S>(tup)...);
}
template <typename... U>
std::string tuple_to_str(const std::tuple<U...>& tup) {
  std::string res;
  append_tuple_call(
      &res, std::forward<const std::tuple<U...>&>(tup), typename gens<sizeof...(U)>::type());
  return res;
}

template <typename V,
          typename W = V,
          typename std::enable_if<is_specialization_of<std::tuple, V>::value>::type* = nullptr>
std::string apply(const W& tup) {
  return tuple_to_str(std::forward<const W&>(tup));
}

// others
/* testing existance of a to_string method */
template <typename Tested>
class has_to_string_method {
  template <typename C>
  static char test(decltype(&C::to_string));
  template <typename C>
  static long test(...);

 public:
  enum { value = sizeof(test<Tested>(nullptr)) == sizeof(char) };
};

// if the class has to_string:
template <
    typename V,
    typename W = V,
    typename std::enable_if<!std::is_arithmetic<V>::value && !std::is_same<V, std::string>::value &&
                            !is_specialization_of<std::tuple, V>::value &&
                            has_to_string_method<V>::value>::type* = nullptr>
std::string apply(const W& val) {
  return val.to_string();
}

// no to string method, fall back to << operator
template <
    typename V,
    typename W = V,
    typename std::enable_if<!std::is_arithmetic<V>::value && !std::is_same<V, std::string>::value &&
                            !is_specialization_of<std::tuple, V>::value &&
                            !has_to_string_method<V>::value>::type* = nullptr>
std::string apply(const W& val) {
  std::stringstream ss;
  ss << val;
  return ss.str();
}

}  // namespace serialize

namespace deserialize {
// arithmetic
template <typename V,
          typename W = V,
          typename std::enable_if<
              std::is_same<V, int>::value || std::is_same<V, short>::value ||
              std::is_same<V, long>::value || std::is_same<V, long long>::value ||
              std::is_same<V, unsigned int>::value || std::is_same<V, unsigned short>::value ||
              std::is_same<V, unsigned long>::value || std::is_same<V, unsigned long long>::value ||
              std::is_same<V, float>::value || std::is_same<V, double>::value ||
              std::is_same<V, long double>::value>::type* = nullptr>
std::pair<bool, W> apply(const std::string& str) {
  if (!isdigit(*str.begin()) && !('-' == *str.begin() && !isdigit(*str.begin())))
    return std::make_pair(false, 0);
  std::istringstream iss(str);
  W res;
  iss >> res;
  return std::make_pair(true, res);
}

// boolean
template <typename V,
          typename W = V,
          typename std::enable_if<std::is_same<V, bool>::value>::type* = nullptr>
std::pair<bool, W> apply(const std::string& str) {
  if ('t' == tolower(*str.begin())) return std::make_pair(true, true);
  if ('f' == tolower(*str.begin())) return std::make_pair(true, false);
  // error:
  return std::make_pair(false, true);
}

// string
template <typename V,
          typename W = V,
          typename std::enable_if<std::is_same<V, std::string>::value>::type* = nullptr>
std::pair<bool, W> apply(const std::string& str) {
  return std::make_pair(true, std::string(str));
}

// char
template <typename V,
          typename W = V,
          typename std::enable_if<std::is_same<V, char>::value>::type* = nullptr>
std::pair<bool, W> apply(const std::string& str) {
  if (!str.empty()) return std::make_pair(true, *str.cbegin());
  return std::make_pair(false, W());
}

template <
    typename V,
    typename W = V,
    typename std::enable_if<std::is_same<V, wchar_t>::value || std::is_same<V, char32_t>::value ||
                            std::is_same<V, char16_t>::value>::type* = nullptr>
std::pair<bool, W> apply(const std::string&) {
  static_assert(true,
                "wchar_t, char16_t and char32_t not supported"
                " by serialize-string.hpp");
  return std::make_pair(false, W());
}

// tuple
template <int, typename... TUP>
bool append_targs(const std::string&, std::tuple<TUP...>*) {
  return true;
}

// FIXME remove int template parameter and use reference for F and U
template <int POS, typename... TUP, typename F, typename... U>
bool append_targs(const std::string& res, std::tuple<TUP...>* tup, F /*first*/, U... args) {
  auto coma_pos = res.find(',');
  auto deserialized = apply<F>(StringUtils::replace_string(
      std::string(res, 0, coma_pos), serialize::tuple_comma_esc_string, ","));
  if (!deserialized.first) return false;
  std::get<POS>(*tup) = deserialized.second;
  // first = deserialized.second;
  auto next_str = std::string();
  if (std::string::npos != coma_pos) next_str = std::string(res, coma_pos + 1);
  return append_targs<POS + 1>(std::forward<const std::string&>(next_str),
                               std::forward<std::tuple<TUP...>*>(tup),
                               std::forward<U>(args)...);
}

template <typename... U, int... S>
bool append_tuple_call(const std::string& res, std::tuple<U...>* tup, tseq<S...>) {
  return append_targs<0>(std::forward<const std::string&>(res),
                         std::forward<std::tuple<U...>*>(tup),
                         std::get<S>(*tup)...);
}

template <typename... U>
bool str_to_tuple(const std::string& str_tup, std::tuple<U...>* tup) {
  return append_tuple_call(std::forward<const std::string&>(str_tup),
                           std::forward<std::tuple<U...>*>(tup),
                           typename gens<sizeof...(U)>::type());
}

template <typename V,
          typename W = V,
          typename std::enable_if<is_specialization_of<std::tuple, V>::value>::type* = nullptr>
std::pair<bool, W> apply(const std::string& str_tup) {
  W tup;
  return std::make_pair(str_to_tuple(std::forward<const std::string&>(str_tup), &tup), tup);
}

// template<typename V, typename W = V,
//          typename std::enable_if<
//            is_specialization_of<std::tuple, V>::value
//            >::type* = nullptr>
// std::pair<bool, W> apply(const std::string &val){
//   // FIXME static_assert<>
//   return str_to_tuple<W>(std::forward<const std::string &>(val));
// }

// other
template <
    typename V,
    typename W = V,
    typename std::enable_if<!std::is_arithmetic<V>::value && !std::is_same<V, std::string>::value &&
                            !is_specialization_of<std::tuple, V>::value>::type* = nullptr>
std::pair<bool, W> apply(const std::string& val) {
  // FIXME static_assert<>
  return V::from_string(std::forward<const std::string&>(val));
}

}  // namespace deserialize

}  // namespace switcher
#endif
