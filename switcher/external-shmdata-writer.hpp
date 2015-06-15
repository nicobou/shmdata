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
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/custom-property-helper.hpp"

namespace switcher {
class ExternalShmdataWriter: public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(ExternalShmdataWriter);
  ExternalShmdataWriter(const std::string &);
  ~ExternalShmdataWriter() = default;
  ExternalShmdataWriter(const ExternalShmdataWriter &) = delete;
  ExternalShmdataWriter &operator=(const ExternalShmdataWriter &) = delete;

 private:
  // custom properties:
  CustomPropertyHelper::ptr custom_props_;
  GParamSpec *shmdata_path_spec_{nullptr};
  std::string shmdata_path_{};
  std::unique_ptr<ShmdataFollower> shm_{nullptr};
  bool init() final;
  static void set_shmdata_path(const gchar *value, void *user_data);
  static const gchar *get_shmdata_path(void *user_data);
};

}  // namespace switcher
#endif
