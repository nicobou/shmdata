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

#ifndef __SWITCHER_QUIDDITY_MANAGER_H__
#define __SWITCHER_QUIDDITY_MANAGER_H__

#include <memory>
#include "./quiddity.hpp"
#include "./switcher.hpp"

namespace switcher {
class Switcher;

class SwitcherWrapper : public Quiddity {
 public:
  SwitcherWrapper() = delete;
  SwitcherWrapper(QuiddityConfiguration&& conf)
      : Quiddity(std::forward<QuiddityConfiguration>(conf)) {}

  typedef std::shared_ptr<SwitcherWrapper> ptr;
  void set_quiddity_manager(std::shared_ptr<Switcher> manager);

 protected:
  std::weak_ptr<Switcher> manager_{};
};

}  // namespace switcher
#endif
