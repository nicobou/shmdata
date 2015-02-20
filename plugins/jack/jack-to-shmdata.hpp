/*
 * This file is part of switcher-jack.
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

#ifndef __SWITCHER_JACK_TO_SHMDATA_H__
#define __SWITCHER_JACK_TO_SHMDATA_H__

#include <memory>
#include "switcher/quiddity.hpp"
#include "switcher/segment.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"
#include "./jack-client.hpp"

namespace switcher {
class JackToShmdata: public Quiddity, public Segment, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(JackToShmdata);
  JackToShmdata(const std::string &);
  ~JackToShmdata() = default;
  JackToShmdata(const JackToShmdata &) = delete;
  JackToShmdata &operator=(const JackToShmdata &) = delete;
  bool start();
  bool stop();

 private:
  CustomPropertyHelper::ptr custom_props_;
  GParamSpec *num_channels_spec_{nullptr};
  uint num_channels_{1};
  GParamSpec *connect_physical_port_prop_{nullptr};
  bool connect_phys_{true};
  GParamSpec *client_name_spec_{nullptr};
  std::string client_name_{};
  JackClient jack_client_;
  bool init() final;

  static void set_num_channels(const gint value, void *user_data);
  static gint get_num_channels(void *user_data);
  static void set_client_name(const gchar *value, void *user_data);
  static const gchar *get_client_name(void *user_data);
  static gboolean get_connect_phys(void *user_data);
  static void set_connect_phys(gboolean connect, void *user_data);
};

SWITCHER_DECLARE_PLUGIN(JackToShmdata);
}  // namespace switcher

#endif
