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

#ifndef __SWITCHER_SHMDATA_CAPS_H__
#define __SWITCHER_SHMDATA_CAPS_H__

#include <string>

#include "switcher/quiddity/quid-id-t.hpp"

namespace switcher {
namespace shmdata {
namespace caps {

std::string get_category(const std::string& caps);

switcher::quiddity::qid_t get_quiddity_id(const std::string& caps);

std::string get_switcher_name(const std::string& caps);

}  // namespace caps
}  // namespace shmdata
}  // namespace switcher
#endif
