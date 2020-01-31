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

#ifndef __SWITCHER_EXTERNAL_SHMDATA_WRITER_H__
#define __SWITCHER_EXTERNAL_SHMDATA_WRITER_H__

#include <memory>
#include "../quiddity/quiddity.hpp"
#include "../shmdata/follower.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class ExternalWriter : public Quiddity {
 public:
  ExternalWriter(quiddity::Config&&);
  ~ExternalWriter() = default;
  ExternalWriter(const ExternalWriter&) = delete;
  ExternalWriter& operator=(const ExternalWriter&) = delete;

 private:
  std::string shmdata_path_{};
  std::unique_ptr<shmdata::Follower> shm_{nullptr};
  InfoTree::ptr on_saving() final;
  void on_loading(InfoTree::ptr&& tree) final;
};

}  // namespace quiddities
}  // namespace switcher
#endif
