/*
 * This file is part of switcher-plugin-example.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_METHOD_PLUGIN_H__
#define __SWITCHER_METHOD_PLUGIN_H__

#include <memory>
#include <string>

#include "switcher/quiddity.hpp"

namespace switcher {
class MethodQuid : public Quiddity {
 public:
  MethodQuid(quid::Config&&);
  ~MethodQuid() = default;
  MethodQuid(const MethodQuid&) = delete;
  MethodQuid& operator=(const MethodQuid&) = delete;

 private:
  MContainer::meth_id_t hello_id_;
  MContainer::meth_id_t count_id_;
  MContainer::meth_id_t many_args_id_;
  using my_method_t = std::function<std::string(std::string)>;
  using many_args_t = std::function<bool(int, float, const std::string&, bool)>;
  std::string hello_{};
  size_t count_{0};
};

SWITCHER_DECLARE_PLUGIN(MethodQuid);

}  // namespace switcher
#endif
