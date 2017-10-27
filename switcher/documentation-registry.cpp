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

#include "documentation-registry.hpp"

namespace switcher {

std::unique_ptr<DocumentationRegistry> DocumentationRegistry::instance_{nullptr};

bool DocumentationRegistry::register_doc(const std::string& quiddity_type,
                                         const QuiddityDocumentation& doc) {
  doc_registry_.emplace(std::make_pair<>(quiddity_type, doc));
  return true;
}

bool DocumentationRegistry::register_type_from_class_name(const std::string& class_name,
                                                          const std::string& quiddity_type) {
  type_from_class_registry_.emplace(std::make_pair<>(class_name, quiddity_type));
  return true;
}

std::string DocumentationRegistry::get_type_from_class_name(const std::string& class_name) {
  auto quiddity_type = type_from_class_registry_.find(class_name);

  if (quiddity_type != type_from_class_registry_.end()) return quiddity_type->second;
  return std::string();
}

const std::map<std::string, QuiddityDocumentation>& DocumentationRegistry::get_docs() {
  return doc_registry_;
}
}
