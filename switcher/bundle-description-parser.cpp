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

#include "./bundle-description-parser.hpp"
#include <regex>

namespace switcher {
namespace bundle {
DescriptionParser::DescriptionParser(const std::string& description,
                                     const std::vector<std::string>& valid_types)
    : is_valid_(parse_description(description, valid_types)) {
  if (!is_valid_) return;

  // check if placeholders in connections_ refer to existing quiddity
  for (auto& it : connections_) {
    std::string placeholder;
    if (!it.src.empty() && *it.src.begin() == '.') {
      // removing the '.'
      placeholder = std::string(it.src.begin() + 1, it.src.end());
      // saving appropriate name instead of placeholder
      it.src = placeholder;
    } else if (!it.sink.empty() && *it.sink.begin() == '.') {
      // removing the '.'
      placeholder = std::string(it.sink.begin() + 1, it.sink.end());
      // saving appropriate name instead of placeholder
      it.sink = placeholder;
    }
    if (placeholder.empty()) continue;
    if (quiddities_.cend() ==
        std::find_if(quiddities_.begin(), quiddities_.end(), [&](const quiddity_spec_t& quid_spec) {
          if (quid_spec.name == placeholder) return true;
          return false;
        })) {
      parsing_error_ =
          std::string("placeholder name ") + placeholder + " is not a valid quiddity name";
      is_valid_ = false;
      return;
    }
  }

  // read connections_ and feed data into quiddities_
  for (auto& it : connections_) {
    auto quid = std::find_if(quiddities_.begin(),
                             quiddities_.end(),
                             [&](const quiddity_spec_t& quid) { return it.src == quid.name; });
    if (quid == quiddities_.end()) {
      parsing_error_ = "BUG during analysis of connections";
      is_valid_ = false;
      return;
    }
    quid->connects_to_.push_back(it.sink);
  }
}

bool DescriptionParser::parse_description(const std::string& description,
                                          const std::vector<std::string>& valid_types) {
  // split by '&', but can be escaped with '\\&'
  std::regex rgx("[^\\\\]&");
  std::sregex_token_iterator iter(description.begin(), description.end(), rgx, -1);
  for (; iter != std::sregex_token_iterator(); ++iter) {
    if (static_cast<std::string>(*iter).empty()) continue;
    if (!parse_branch(*iter, valid_types)) return false;
  }
  return true;
}

bool DescriptionParser::parse_branch(const std::string& branch,
                                     const std::vector<std::string>& valid_types) {
  // split by '!', but can be escaped with '\\!'
  std::regex rgx("[^\\\\]!");
  std::sregex_token_iterator iter(branch.begin(), branch.end(), rgx, -1);
  for (; iter != std::sregex_token_iterator(); ++iter) {
    if (static_cast<std::string>(*iter).empty()) continue;
    if (!parse_item(*iter, valid_types)) return false;
  }
  previous_quid_.clear();
  return true;
}

bool DescriptionParser::parse_item(const std::string& raw_item,
                                   const std::vector<std::string>& valid_types) {
  // start with some filtering
  std::string item = prepare_item(raw_item);
  item = protect_space_in_quote(item);
  // split by space' ', but can be escaped with '\\ '
  std::regex rgx("\\s");
  std::sregex_token_iterator iter(item.begin(), item.end(), rgx, -1);
  bool type_found = false;
  static const std::string shmr("_shmr");
  static const std::string shmw("_shmw");
  static const std::string noprop("_no_prop");
  quiddity_spec_t quid;
  for (; iter != std::sregex_token_iterator(); ++iter) {
    if (static_cast<std::string>(*iter).empty()) continue;
    // first thing to do is to find quiddity type
    if (!type_found) {
      const auto& str = static_cast<std::string>(*iter);
      if (*str.begin() == '.') {
        if (str.size() == 1) {
          parsing_error_ = "wrong placeholder name (\".\")";
          return false;
        }
        // save the name with the '.' for later check
        if (!previous_quid_.empty()) {
          if (*previous_quid_.begin() == '.') {
            parsing_error_ = "placeholders cannot be connected to each others (" + previous_quid_ +
                             " and " + str + ")";
            return false;
          }
          connections_.push_back(shm_connection_t(previous_quid_, str));
        }
        previous_quid_ = str;
        return true;
      } else {
        if (!valid_types.empty() &&
            valid_types.end() == std::find(valid_types.begin(), valid_types.end(), *iter)) {
          parsing_error_ = static_cast<std::string>(*iter) + " is not a valid quiddity type";
          return false;
        }
        quid.type = *iter;
      }
      type_found = true;
      continue;
    }
    if (*iter == shmr) {
      if (!reader_quid_.empty()) {
        std::string current = quid.name.empty() ? "an other unknown quiddity" : quid.name;
        parsing_error_ = std::string("bundle description can get only one shmdata reader (") +
                         reader_quid_ + " and " + current + " are labelled as shmreader)";
        return false;
      }
      quid.expose_shmr = true;
      continue;
    }
    if (*iter == shmw) {
      quid.expose_shmw = true;
      continue;
    }
    if (*iter == noprop) {
      quid.expose_prop = false;
      continue;
    }
    if (!parse_param(*iter, quid)) return false;
  }

  if (quiddities_.end() !=
      std::find_if(quiddities_.begin(), quiddities_.end(), [&](const quiddity_spec_t& quiddity) {
        return quiddity.name == quid.name;
      })) {
    parsing_error_ =
        std::string("quiddity name ") + quid.name + " is not unique in bundle description";
    return false;
  }
  if (quid.name.empty()) {
    parsing_error_ =
        std::string("quiddity name is missing for quiddity specified with type ") + quid.type;
    return false;
  }
  if (!previous_quid_.empty()) connections_.push_back(shm_connection_t(previous_quid_, quid.name));
  if (quid.expose_shmr) reader_quid_ = quid.name;
  previous_quid_ = quid.name;
  quiddities_.push_back(quid);
  return true;
}

bool DescriptionParser::parse_param(const std::string& raw_param, quiddity_spec_t& quid) {
  std::string param = restore_space_in_quote(raw_param);
  std::regex rgx("\\w+=[^=]+");
  if (!std::regex_match(param, rgx)) {
    parsing_error_ =
        std::string("wrong parameter syntax (expecting param_name=param_value, but got ") + param;
    return false;
  }
  auto equal_pos = param.find('=');
  auto key = std::string(param, 0, equal_pos);
  if (key == "name")
    quid.name = std::string(param, equal_pos + 1, std::string::npos);
  else
    quid.params[key] = std::string(param, equal_pos + 1, std::string::npos);
  return true;
}

std::string DescriptionParser::protect_space_in_quote(const std::string& item) const {
  std::string res = item;
  std::regex rgx("(\\\"[^\\\"]*)\\s([^\\\"]*\\\")");
  while (std::regex_search(res, rgx))
    res = std::regex_replace(res, rgx, std::string("$1") + spaceReplacement + "$2");
  return res;
}

std::string DescriptionParser::restore_space_in_quote(const std::string& param) const {
  std::regex rgx(spaceReplacement);
  return std::regex_replace(param, rgx, " ");
}

std::string DescriptionParser::prepare_item(const std::string& item) const {
  // no space around '='
  std::regex rgx("\\s*=\\s*");
  std::string filtered = std::regex_replace(item, rgx, "=");
  return filtered;
}

}  // namespace bundle
}  // namespace switcher
