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

#ifndef __SWITCHER_BUNDLE_H__
#define __SWITCHER_BUNDLE_H__

#include "./quiddity.hpp"

namespace switcher {
class Bundle : public Quiddity {
 public:
  using doc_getter_t = std::function<QuiddityDocumentation*()>;
  Bundle(const std::string&);
  ~Bundle() = default;
  Bundle(const Bundle&) = delete;
  Bundle& operator=(const Bundle&) = delete;
  bool init() final;
  QuiddityDocumentation* get_documentation() final;
  void set_doc_getter(doc_getter_t doc_getter);

 private:
  std::string pipeline_{};
  doc_getter_t doc_getter_{};
};

// wrappers for the abstract factory registration
namespace bundle {
Quiddity* create(const std::string& name);
void destroy(Quiddity* quiddity);
}  // namespace bundle

}  // namespace switcher
#endif
