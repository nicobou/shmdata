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

#ifndef SWITCHER_DOCUMENTATION_REGISTRY_HPP
#define SWITCHER_DOCUMENTATION_REGISTRY_HPP

#include <map>
#include <memory>
#include "./quiddity-documentation.hpp"

namespace switcher {

class DocumentationRegistry {
 public:
  static std::unique_ptr<DocumentationRegistry>& get() {
    if (!instance_) instance_ = std::make_unique<DocumentationRegistry>();
    return instance_;
  }
  bool register_doc(const std::string& quiddity_type, const QuiddityDocumentation& doc);
  bool register_type_from_class_name(const std::string& class_name, const std::string& quid_type);
  std::string get_type_from_class_name(const std::string& class_name);
  const std::map<std::string, QuiddityDocumentation>& get_docs();

 private:
  static std::unique_ptr<DocumentationRegistry> instance_;
  std::map<std::string, QuiddityDocumentation> doc_registry_{};
  std::map<std::string, std::string> type_from_class_registry_{};
};
}
#endif
