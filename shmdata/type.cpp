/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#include "./type.hpp"
#include <regex>
#include <string>
#include <sstream>

namespace shmdata {

Type::Type(const std::string& str){
  auto type = escape(str);
  // split by ','
  std::regex rgx(" *, *");
  std::sregex_token_iterator iter(type.begin(), type.end(), rgx, -1);
  // searching for the type name
  while (name_.empty() && iter != std::sregex_token_iterator()) {
    name_ = static_cast<std::string>(*iter);
    ++iter;
  }
  // 
  for (; iter != std::sregex_token_iterator(); ++iter) {
    auto prop = static_cast<std::string>(*iter);
    if (prop.empty()) continue;
    // removing space around '='
    std::regex space_rgx("\\s*=\\s*");
    prop = std::regex_replace(prop, space_rgx, "=");
    // ensure we have a 'key=value' description 
    std::regex keyval_rgx("[\\w-]+=[^=]+");
    if (!std::regex_match(prop, keyval_rgx)) {
      parsing_errors_ +=
          std::string("wrong parameter syntax (expecting param_name=param_value, but got ") + static_cast<std::string>(*iter);
      continue;
    }
    // key and value
    auto equal_pos = prop.find('=');
    auto key = std::string(prop, 0, equal_pos);
    std::string value = std::string(prop, equal_pos + 1, std::string::npos);
    // check if value contains a literal type description
    std::regex literal_type_regex("\\([\\w]+\\)");
    std::smatch sm;
    auto unescaped = unescape(value);
    // check if unescaped value is an integer and fill the any value with the appropriate type
    std::any any_value;
    bool is_negative = false;
    if (*unescaped.begin() == '-') is_negative = true;
    if (!unescaped.empty() && unescaped != "-" &&
        std::find_if(unescaped.begin() + (is_negative ? 1 : 0), unescaped.end(), [](char c) {
          return !std::isdigit(c);
        }) == unescaped.end()) {
      any_value = std::stoi(unescaped);
    } else {
      any_value = std::make_any<std::string>(unescaped);
    }
    // search if the value is prefixed with a type name
    if (regex_search(value, sm, literal_type_regex)) {
      if (sm.str() == "(int)") {
        std::istringstream iss(std::string(value, sm.str().size(), std::string::npos));
        int tmp_int;
        iss >> tmp_int;
        any_value = tmp_int;
      } else {
        any_value = unescape(std::string(value, sm.str().size(), std::string::npos));
        // saving non literal types only
        str_types_.emplace(key, std::string(sm.str(), 1, sm.str().size() - 2));
      }
    } 
    // saving resulting key and value
    properties_.emplace(key, any_value);
  }
}

std::string Type::name() const{
  return name_;
}

std::string Type::get_parsing_errors() const { return parsing_errors_; }

std::any Type::get(const std::string& key) const {
  auto found = properties_.find(key);
  if (properties_.end() == found) return std::any();
  return found->second;
}

void Type::set_prop(const std::string& key, const char* value) {
  properties_.emplace(key, std::string(value));
}

void Type::set_prop(const std::string& key,
                    const std::string& custom_type,
                    const std::string& value) {
  properties_.emplace(key,value);
  str_types_.emplace(key,custom_type);
}

std::string Type::escape(const std::string& str) {
  std::regex coma_rgx("\\\\,");
  std::regex equal_rgx("\\\\=");
  return std::regex_replace(std::regex_replace(str, coma_rgx, "__COMA__"), equal_rgx, "__EQUAL__");
}

std::string Type::unescape(const std::string& str) {
  std::regex coma_rgx("__COMA__");
  std::regex equal_rgx("__EQUAL__");
  std::regex quot_rgx("\\\"");
  std::regex lparent_rgx("\\\\\\(");
  std::regex rparent_rgx("\\\\\\)");
  return std::regex_replace(
      std::regex_replace(
          std::regex_replace(
              std::regex_replace(std::regex_replace(str, coma_rgx, ","), quot_rgx, ""),
              equal_rgx,
              "="),
          lparent_rgx,
          "("),
      rparent_rgx,
      ")");
}

std::map<std::string,std::any> Type::get_properties() const {
  return properties_;
}

std::string Type::get_serialized_string_value(const std::any& value) {
  std::regex coma_rgx(",");
  std::regex equal_rgx("=");
  std::regex lparent_rgx("\\(");
  std::regex rparent_rgx("\\)");
  auto res = std::regex_replace(
      std::regex_replace(
          std::regex_replace(std::regex_replace(std::any_cast<std::string>(value), coma_rgx, "\\,"),
                             equal_rgx,
                             "\\="),
          lparent_rgx,
          "\\("),
      rparent_rgx,
      "\\)");
  // add quote if space, coma of equal is present in the string
  if (res.find(' ') != std::string::npos || res.find(',') != std::string::npos ||
      res.find('=') != std::string::npos)
    res = "\"" + res + "\"";
  return res;
}

std::string Type::str() const {
  auto res = name_;
  for (auto& it : properties_) {
    auto value = it.second;
    auto found = str_types_.find(it.first);
    if (found != str_types_.end()) {
      res += ", " + it.first + "=(" + found->second + ")" + get_serialized_string_value(value);
    } else {
      auto thc = value.type().hash_code();
      if (typeid(std::string).hash_code() == thc) {
        res += ", " + it.first + "=(string)" + get_serialized_string_value(value);
      } else if (typeid(int).hash_code() == thc) {
        res += ", " + it.first + "=(int)" + std::to_string(std::any_cast<int>(value));
      } else if (typeid(long).hash_code() == thc) {
        res += ", " + it.first + "=(long)" + std::to_string(std::any_cast<long>(value));
      } else if (typeid(long long).hash_code() == thc) {
        res += ", " + it.first + "=(long long)" + std::to_string(std::any_cast<long long>(value));
      } else if (typeid(unsigned).hash_code() == thc) {
        res += ", " + it.first + "=(unsigned)" + std::to_string(std::any_cast<unsigned>(value));
      } else if (typeid(unsigned long).hash_code() == thc) {
        res += ", " + it.first + "=(unsigned long)" + std::to_string(std::any_cast<unsigned long>(value));
      } else if (typeid(unsigned long long).hash_code() == thc) {
        res += ", " + it.first + "=(unsigned long long)" + std::to_string(std::any_cast<unsigned long long>(value));
      } else if (typeid(float).hash_code() == thc) {
        res += ", " + it.first + "=(float)" + std::to_string(std::any_cast<float>(value));
      } else if (typeid(double).hash_code() == thc) {
        res += ", " + it.first + "=(double)" + std::to_string(std::any_cast<double>(value));
      } else if (typeid(long double).hash_code() == thc) {
        res += ", " + it.first + "=(long double)" + std::to_string(std::any_cast<long double>(value));
      } else {
        serialization_errors_ += std::string("unknown type for key ") + it.first +
            " cpp type name is " + value.type().name();
      }
    }
  }
  return res;
}

}  // namespace shmdata
