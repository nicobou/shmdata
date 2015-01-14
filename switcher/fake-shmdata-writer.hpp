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

#ifndef __SWITCHER_FAKE_SHMDATA_WRITER_H__
#define __SWITCHER_FAKE_SHMDATA_WRITER_H__

#include <memory>
#include "./quiddity.hpp"
#include "./segment.hpp"
#include "./startable-quiddity.hpp"
#include "./custom-property-helper.hpp"
#include "./quiddity-manager.hpp"

namespace switcher {
class FakeShmdataWriter: public Quiddity,
                         public Segment,
                         public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(FakeShmdataWriter);
  FakeShmdataWriter(const std::string &);
  ~FakeShmdataWriter();
  FakeShmdataWriter(const FakeShmdataWriter &) = delete;
  FakeShmdataWriter &operator=(const FakeShmdataWriter &) = delete;

  bool add_shmdata_path(std::string name);
  bool start() final;
  bool stop() final;

 private:
  // custom properties:
  CustomPropertyHelper::ptr custom_props_;
  GParamSpec *shmdata_path_spec_{nullptr};
  std::string shmdata_path_{"none"};
  std::string caps_{};
  QuiddityManager::ptr manager_{};
  // getting caps from fakesink
  static void caps_cb(std::string /*subscriber_name */ ,
                      std::string /*quiddity_name*/,
                      std::string /*property_name*/,
                      std::string value,
                      void *user_data);
  bool init() final;  // segment implementation
  static gboolean add_shmdata_path_wrapped(gpointer name,
                                           gpointer user_data);
  static void set_shmdata_path(const gchar *value, void *user_data);
  static const gchar *get_shmdata_path(void *user_data);
};

}  // namespace switcher
#endif
