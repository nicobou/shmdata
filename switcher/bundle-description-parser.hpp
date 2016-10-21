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

#ifndef __SWITCHER_BUNDLE_DESCRIPTION_PARSER_H__
#define __SWITCHER_BUNDLE_DESCRIPTION_PARSER_H__

#include <map>
#include <string>
#include <vector>
#include "./safe-bool-idiom.hpp"

namespace switcher {
namespace bundle {
struct quiddity_spec_t {
  std::string type{};
  std::string name{};
  std::map<std::string, std::string> params{};
  bool expose_shmr{false};
  bool expose_shmw{false};
  std::vector<std::string> connects_to_{};
  std::vector<std::string> connected_to_{};
};

struct shm_connection_t {
  shm_connection_t() = delete;
  shm_connection_t(const std::string& sc, const std::string& sk) : src(sc), sink(sk){};
  std::string src{};
  std::string sink{};
};

class DescriptionParser : public SafeBoolIdiom {
 public:
  DescriptionParser() = delete;
  // if valid_types is empty, then no check is performed
  DescriptionParser(const std::string& description, const std::vector<std::string>& valid_types);
  std::string get_parsing_error() const { return parsing_error_; }
  std::vector<quiddity_spec_t> get_quiddities() const { return quiddities_; }
  std::string get_reader_quid() const { return reader_quid_; }

 private:
  std::string previous_quid_{};
  std::string parsing_error_{};
  const std::string spaceReplacement{"__SPACE__"};
  std::vector<quiddity_spec_t> quiddities_{};
  std::vector<shm_connection_t> connections_{};
  std::string reader_quid_{};
  bool is_valid_;
  bool parse_description(const std::string& description,
                         const std::vector<std::string>& valid_types);
  bool parse_branch(const std::string& branch, const std::vector<std::string>& valid_types);
  bool parse_item(const std::string& item, const std::vector<std::string>& valid_types);
  bool parse_param(const std::string& raw_param, quiddity_spec_t& quid);
  std::string protect_space_in_quote(const std::string& item) const;
  std::string restore_space_in_quote(const std::string& param) const;
  std::string prepare_item(const std::string& item) const;
  bool safe_bool_idiom() const final { return is_valid_; };
};
}  // namespace bundle
}  // namespace switcher

#endif
