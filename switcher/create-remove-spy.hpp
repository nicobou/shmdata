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

#ifndef __SWITCHER_CREATE_REMOVE_SPY_H__
#define __SWITCHER_CREATE_REMOVE_SPY_H__

#include <memory>
#include "./quiddity.hpp"

namespace switcher {
class CreateRemoveSpy: public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(CreateRemoveSpy);
  CreateRemoveSpy(const std::string &);
  ~CreateRemoveSpy();
  CreateRemoveSpy(const CreateRemoveSpy &) = delete;
  CreateRemoveSpy &operator=(const CreateRemoveSpy &) = delete;
  bool init();
  static void on_created(const std::string &quiddity_nick_name, void *user_data);
  static void on_removed(const std::string &quiddity_nick_name, void *user_data);

 private:
  bool i_am_the_one_;
};

}  // namespace switcher
#endif
