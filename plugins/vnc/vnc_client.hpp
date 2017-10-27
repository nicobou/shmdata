/*
 * This file is part of posture.
 *
 * posture is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_VNCCLIENT_SRC_H__
#define __SWITCHER_VNCCLIENT_SRC_H__

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

#include <rfb/rfbclient.h>

#define VNC_MOUSE_EVENTS_CAPS "application/x-mouse-events"
#define VNC_KEYBOARD_EVENTS_CAPS "application/x-keyboard-events"

namespace switcher {
class VncClientSrc : public Quiddity, public StartableQuiddity {
 public:
  VncClientSrc(QuiddityConfiguration&&);
  ~VncClientSrc();
  VncClientSrc(const VncClientSrc&) = delete;
  VncClientSrc& operator=(const VncClientSrc&) = delete;

  bool start();
  bool stop();

 private:
  ShmdataConnector shmcntr_;
  // prop
  std::string vnc_server_address_{"localhost"};
  PContainer::prop_id_t vnc_server_address_id_{0};
  PContainer::prop_id_t capture_truecolor_id_{0};
  bool capture_truecolor_{true};
  bool previous_truecolor_state_{true};
  rfbClient* rfb_client_{nullptr};
  std::vector<unsigned char> framebuffer_{};
  size_t framebuffer_size_{0};
  std::unique_ptr<ShmdataWriter> vnc_writer_{nullptr};
  std::atomic_bool vnc_continue_update_{false};
  std::thread vnc_update_thread_{};

  std::mutex mutex_{};
  std::mutex connect_mutex_{};
  int shmreader_id_{0};
  std::map<int, std::string> shmdata_readers_caps_{};
  std::map<std::string, std::unique_ptr<ShmdataFollower>> events_readers_{};

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string shmName);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);

  static rfbBool resize_vnc(rfbClient* client);
  static void update_vnc(rfbClient* client, int x, int y, int w, int h);
};

SWITCHER_DECLARE_PLUGIN(VncClientSrc);
}  // namespace switcher
#endif
