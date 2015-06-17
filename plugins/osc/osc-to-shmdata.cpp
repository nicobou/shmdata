/*
 * This file is part of switcher-osc.
 *
 * switcher-osc is free software; you can redistribute it and/or
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

#include "switcher/std2.hpp"
#include "./osc-to-shmdata.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    OscToShmdata,
    "OSCsrc",
    "OSC Receiver",
    "network",
    "writer",
    "receives OSC messages and write to shmdata",
    "LGPL",
    "Nicolas Bouillot");

OscToShmdata::OscToShmdata(const std::string &):
    custom_props_(new CustomPropertyHelper()),
    port_(1056),
    osc_thread_(nullptr),
    port_spec_(nullptr) {
}

bool OscToShmdata::init() {
  init_startable(this);
  port_spec_ =
      custom_props_->make_int_property("Port",
                                       "OSC port to listen",
                                       1,
                                       65536,
                                       port_,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       set_port, get_port, this);

  install_property_by_pspec(custom_props_->get_gobject(),
                            port_spec_, "port", "Port");

  return true;
}

OscToShmdata::~OscToShmdata() {
  stop();
}

void OscToShmdata::set_port(const gint value, void *user_data) {
  OscToShmdata *context = static_cast<OscToShmdata *>(user_data);
  context->port_ = value;
}

gint OscToShmdata::get_port(void *user_data) {
  OscToShmdata *context = static_cast<OscToShmdata *>(user_data);
  return context->port_;
}

bool OscToShmdata::start() {
  // creating a shmdata
  shm_ = std2::make_unique<ShmdataWriter>(this,
                                          make_file_name("osc"),
                                          4096,
                                          "application/x-libloserialized-osc");
  if(!shm_.get()) {
    g_warning("OscToShmdata failed to start");
    shm_.reset(nullptr);
    return false;
  }
  
  osc_thread_ =
      lo_server_thread_new(std::to_string(port_).c_str(), osc_error);
  if (nullptr == osc_thread_)
    return false;
  /* add method that will match any path and args */
  lo_server_thread_add_method(osc_thread_, nullptr, nullptr, osc_handler,
                              this);
  lo_server_thread_start(osc_thread_);
  return true;
}

bool OscToShmdata::stop() {
  shm_.reset(nullptr);
  if (nullptr != osc_thread_) {
    lo_server_thread_free(osc_thread_);
    osc_thread_ = nullptr;
    return true;
  }
  return false;
}

/* catch any osc incoming messages. */
int
OscToShmdata::osc_handler(const char *path,
                          const char */*types*/,
                          lo_arg **/*argv*/,
                          int /*argc*/,
                          lo_message m,
                          void *user_data) {
  OscToShmdata *context = static_cast<OscToShmdata *>(user_data);
  lo_timetag timetag = lo_message_get_timestamp(m);
  // g_print ("timestamp %u %u", path, timetag.sec, timetag.frac);
  if (0 != timetag.sec) {
    // FIXME handle internal timetag
    // note: this is not implemented in osc-send
  }
  size_t size;
  void *buftmp = lo_message_serialise(m, path, nullptr, &size);
  if (context->shm_->writer(&shmdata::Writer::alloc_size) < size) {
    context->shm_.reset(nullptr);
    context->shm_.reset(new ShmdataWriter(context,
                                          context->make_file_name("osc"),
                                          size,
                                          "application/x-libloserialized-osc"));
  }
  context->shm_->writer(&shmdata::Writer::copy_to_shm, buftmp, size);
  context->shm_->bytes_written(size);
  g_free(buftmp);
  return 0;
}

void OscToShmdata::osc_error(int num, const char *msg, const char *path) {
  g_debug("liblo server error %d in path %s: %s", num, path, msg);
}
}                               // end of OscToShmdata class
