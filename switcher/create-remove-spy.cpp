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

#include "./create-remove-spy.hpp"
#include "./gst-utils.hpp"
#include "./quiddity-manager-impl.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    CreateRemoveSpy,
    "create_remove_spy",
    "Quiddity Creation Inspector",
    "utils",
    "",
    "spy manager for quidity creation and removal and convert into signals",
    "LGPL",
    "Nicolas Bouillot");

CreateRemoveSpy::CreateRemoveSpy(const std::string &):
    i_am_the_one_(false) {
}

bool CreateRemoveSpy::init() {
  QuiddityManager_Impl::ptr manager = manager_impl_.lock();
  if ((bool) manager) {
    if (!manager->set_created_hook(CreateRemoveSpy::on_created, this))
      return false;
    if (!manager->set_removed_hook(CreateRemoveSpy::on_removed, this))
      return false;
  }
  else
    return false;

  i_am_the_one_ = true;

  // we got the hook, so make signals of it
  GType string_type[] = { G_TYPE_STRING };
  install_signal("On Quiddity Created",
                 "on-quiddity-created",
                 "a quiddity has been created",
                 Signal::make_arg_description("Quiddity Name",
                                              "quiddity_name",
                                              "the quiddity name",
                                              nullptr),
                 1,
                 string_type);

  install_signal("On Quiddity Removed",
                 "on-quiddity-removed",
                 "a quiddity has been removed",
                 Signal::make_arg_description("Quiddity Name",
                                              "quiddity_name",
                                              "the quiddity name",
                                              nullptr),
                 1,
                 string_type);
  return true;
}

CreateRemoveSpy::~CreateRemoveSpy() {
  if (i_am_the_one_) {
    QuiddityManager_Impl::ptr manager = manager_impl_.lock();
    if ((bool) manager)
      manager->reset_create_remove_hooks();
  }
}

void
CreateRemoveSpy::on_created(const std::string &quiddity_nick_name,
                            void *user_data) {
  CreateRemoveSpy *context = static_cast<CreateRemoveSpy *>(user_data);
  context->signal_emit("on-quiddity-created", quiddity_nick_name.c_str());
}

void
CreateRemoveSpy::on_removed(const std::string &quiddity_nick_name,
                            void *user_data) {
  CreateRemoveSpy *context = static_cast<CreateRemoveSpy *>(user_data);
  context->signal_emit("on-quiddity-removed", quiddity_nick_name.c_str());
}
}
