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

#ifndef __SWITCHER_QUIDDITY_PROXID_H__
#define __SWITCHER_QUIDDITY_PROXID_H__

#include <string>
#include "../utils/bool-log.hpp"
#include "./quid-id-t.hpp"
#include "./quiddity.hpp"

namespace switcher {
namespace quid {
class Qrox : public BoolLog {
 public:
  Qrox();
  Qrox(const Qrox& cpyqrox) = default;
  Qrox(bool is_valid);
  Qrox(bool is_valid, const std::string& msg);
  Qrox(bool is_valid, const std::string& msg, qid_t id, Quiddity* ptr);
  qid_t get_id() const;
  Quiddity* get();

 private:
  qid_t id_{0};
  Quiddity* quid_{nullptr};
};

}  // namespace quid
}  // namespace switcher
#endif
