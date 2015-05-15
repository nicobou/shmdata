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

#include "./fakesink.hpp"
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(FakeSink,
                                     "Shmdata Inspector",
                                     "monitor",
                                     "monitoring a shmdata writer",
                                     "LGPL",
                                     "fakesink",
                                     "Nicolas Bouillot");

FakeSink::FakeSink(const std::string &):
    fakesink_("fakesink"),
    shmcntr_(static_cast<Quiddity *>(this)),
    props_(std::make_shared<CustomPropertyHelper>()){
}

FakeSink::~FakeSink() {
  if (update_byterate_source_ != nullptr)
    g_source_destroy(update_byterate_source_);
}

bool FakeSink::init() {
  if (!fakesink_)
    return false;
  g_object_set(G_OBJECT(fakesink_.get_raw()),
               "sync", FALSE,
               "signal-handoffs", FALSE,
               nullptr);
  shmcntr_.install_connect_method([](const std::string &){
      return true;
    },// FIXME connect,
    nullptr, //no disconnect
    [](){return true;},// FIXME disconnectall
    [](const std::string &){return true;},
    1);
  return true;
}

}  // namespace switcher
