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

#include <string>
#include <utility>
#include <cctype>  // tolower

namespace switcher {
namespace serialize { 
// arithmetic
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, int>::value
           || std::is_same<V, short>::value
           || std::is_same<V, long>::value
           || std::is_same<V, long long>::value
           || std::is_same<V, unsigned int>::value
           || std::is_same<V, unsigned short>::value
           || std::is_same<V, unsigned long>::value
           || std::is_same<V, unsigned long long>::value
           || std::is_same<V, float>::value
           || std::is_same<V, double>::value
           || std::is_same<V, long double>::value
           >::type* = nullptr>
    std::string apply(const W &val){
  return std::to_string(val);
} 

// boolean
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, bool>::value
           >::type* = nullptr>
std::string apply(const W &val){
  if (val)
    return std::string("true");
  return std::string("false");
} 

// string
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, std::string>::value
           >::type* = nullptr>
std::string apply(const W &val){
  return std::string(val);
} 

// char
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, char>::value
           >::type* = nullptr>
    std::string apply(const W &val){
  return std::string(&val);
} 

template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, wchar_t>::value
           || std::is_same<V, char32_t>::value
           || std::is_same<V, char16_t>::value
           >::type* = nullptr>
    std::string apply(const W &){
  static_assert(
      true, "wchar_t, char16_t and char32_t not supported by serialize-string.hpp");
  return std::string();
}

// others 
template<typename V, typename W = V,
         typename std::enable_if<
           !std::is_arithmetic<V>::value
           && !std::is_same<V, std::string>::value
           >::type* = nullptr>  
std::string apply(const W &val){
  // FIXME static_assert existance of to_string member
  return val.to_string();
}
}  // namespace serialize 

namespace deserialize{
// arithmetic
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, int>::value
           || std::is_same<V, short>::value
           || std::is_same<V, long>::value
           || std::is_same<V, long long>::value
           || std::is_same<V, unsigned int>::value
           || std::is_same<V, unsigned short>::value
           || std::is_same<V, unsigned long>::value
           || std::is_same<V, unsigned long long>::value
           || std::is_same<V, float>::value
           || std::is_same<V, double>::value
           || std::is_same<V, long double>::value
           >::type* = nullptr>
std::pair<bool, W> apply(const std::string &str){
  if (!isdigit(*str.begin())
      && !('-' == *str.begin() && !isdigit(*str.begin())))
    return std::make_pair(false, 0);
  std::istringstream iss(str);
  W res;
  iss >> res;
  return std::make_pair(true,res);
}

// boolean
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, bool>::value
           >::type* = nullptr>
std::pair<bool, W> apply(const std::string &str){
  if ('t' == tolower(*str.begin()))
    return std::make_pair(true, true);
  if ('f' == tolower(*str.begin()))
    return std::make_pair(true, false);
  // error:
  return std::make_pair(false, true);
} 

// string
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, std::string>::value
           >::type* = nullptr>
std::pair<bool, W> apply(const std::string &str){
  return std::make_pair(true, std::string(str));
} 

// char
template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, char>::value
           >::type* = nullptr>
std::pair<bool, W> apply(const std::string &str){
  if (!str.empty())
    return std::make_pair(true, *str.cbegin());
  return std::make_pair(false, W());
} 

template<typename V, typename W = V,
         typename std::enable_if<
           std::is_same<V, wchar_t>::value
           || std::is_same<V, char32_t>::value
           || std::is_same<V, char16_t>::value
           >::type* = nullptr>
std::pair<bool, W> apply(const std::string &){
  static_assert(true,
                "wchar_t, char16_t and char32_t not supported"
                " by serialize-string.hpp");
  return std::make_pair(false, W());
} 

// other
template<typename V, typename W = V,
         typename std::enable_if<
           !std::is_arithmetic<V>::value
           && !std::is_same<V, std::string>::value
           >::type* = nullptr>  
std::pair<bool, W> apply(const std::string &val){
  // FIXME static_assert<>
  return V::from_string(std::forward<const std::string &>(val));
}
}  // namespace deserialize

}  // namespace switcher
#endif
