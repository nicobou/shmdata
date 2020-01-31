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

#ifndef __SWITCHER_JACK_SERVER_QUID_H__
#define __SWITCHER_JACK_SERVER_QUID_H__

#include <memory>
#include "./jack-server.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/quiddity/startable.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class JackServerQuid : public Quiddity, public Startable {
 public:
  JackServerQuid(quiddity::Config&&);

 private:
  JackServer jack_server_;
  // config_ is a shared ptr obtained from jack_server_.
  InfoTree::ptr config_;
  property::prop_id_t driver_config_id_;
  property::prop_id_t advanced_config_id_;
  // driver properties
  property::Selection<> driver_enum_;
  property::prop_id_t driver_id_{0};

  std::vector<property::prop_id_t> driver_params_{};
  std::map<std::string, property::Selection<>> selections_{};

  property::prop_id_t make_param(const std::string& config_path, const std::string& parent);
  void renew_driver_properties();
  bool start() final;
  bool stop() final;
};

SWITCHER_DECLARE_PLUGIN(JackServerQuid);

}  // namespace quiddities
}  // namespace switcher
#endif
