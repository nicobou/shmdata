/*
 * This file is part of switcher-recplay.
 *
 * switcher-recplay is free software; you can redistribute it and/or
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

#ifndef SWITCHER_AVPLAYER_HPP
#define SWITCHER_AVPLAYER_HPP

#include <gst/gst.h>
#include <string.h>
#include <switcher/startable-quiddity.hpp>
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"

namespace switcher {
class AVPlayer : public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(AVPlayer);
  AVPlayer(const std::string& name);
  bool init() { return true; };
  bool start() { return true; }
  bool stop() { return true; }
};

SWITCHER_DECLARE_PLUGIN(AVPlayer);
}  // namespace switcher

#endif
