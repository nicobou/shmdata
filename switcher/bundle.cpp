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

#include "./bundle.hpp"
#include "./bundle-description-parser.hpp"

namespace switcher {

Bundle::Bundle(const std::string& name) : manager_(QuiddityManager::make_manager(name)) {}

bool Bundle::init() {
  if (!config<MPtr(&InfoTree::branch_has_data)>("pipeline")) {
    g_warning("bundle description is missing the pipeline description");
    return false;
  }
  pipeline_ = config<MPtr(&InfoTree::branch_get_value)>("pipeline").copy_as<std::string>();
  auto spec = bundle::DescriptionParser(pipeline_, std::vector<std::string>());
  if (!spec) {
    g_warning("%s : error parsing the pipeline (%s)",
              doc_getter_()->get_class_name().c_str(),
              spec.get_parsing_error().c_str());
    return false;
  }
  return true;
}

QuiddityDocumentation* Bundle::get_documentation() {
  if (doc_getter_) return doc_getter_();
  return nullptr;
}

void Bundle::set_doc_getter(doc_getter_t doc_getter) { doc_getter_ = doc_getter; }

namespace bundle {
Quiddity* create(const std::string& name) { return new Bundle(name); }
void destroy(Quiddity* quiddity) { delete quiddity; }
}

}  // namespace switcher
