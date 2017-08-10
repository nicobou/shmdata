/*
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

#include "./vnc_client.hpp"

#include <cstdint>
#include <functional>
#include <iostream>

using namespace std;

namespace switcher {

SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(VncClientSrc,
                                     "vncclientsrc",
                                     "VNC client",
                                     "video",
                                     "writer/reader",
                                     "Connects to a VNC server and outputs the video to a shmdata",
                                     "LGPL",
                                     "Emmanuel Durand");

VncClientSrc::VncClientSrc(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  init_startable(this);

  shmcntr_.install_connect_method([this](const std::string path) { return connect(path); },
                                  [this](const std::string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const std::string caps) { return can_sink_caps(caps); },
                                  2);
  vnc_server_address_id_ =
      pmanage<MPtr(&PContainer::make_string)>("vnc_server_address",
                                              [this](const std::string& val) {
                                                vnc_server_address_ = val;
                                                return true;
                                              },
                                              [this]() { return vnc_server_address_; },
                                              "IP address",
                                              "Address of the VNC server",
                                              vnc_server_address_);
  capture_truecolor_id_ =
      pmanage<MPtr(&PContainer::make_bool)>("capture_truecolor",
                                            [this](const bool& val) {
                                              capture_truecolor_ = val;
                                              return true;
                                            },
                                            [this]() { return capture_truecolor_; },
                                            "Capture color depth",
                                            "Capture in 32bits if true, 16bits otherwise",
                                            capture_truecolor_);
}

VncClientSrc::~VncClientSrc() { stop(); }

bool VncClientSrc::start() {
  if (capture_truecolor_)
    rfb_client_ = rfbGetClient(8, 3, 4);
  else
    rfb_client_ = rfbGetClient(5, 3, 2);

  if (!rfb_client_) return false;

  rfbClientSetClientData(
      rfb_client_, (void*)(&VncClientSrc::resize_vnc) /*pointer as a tag, never called*/, this);
  rfb_client_->MallocFrameBuffer = VncClientSrc::resize_vnc;
  rfb_client_->canHandleNewFBSize = TRUE;
  rfb_client_->GotFrameBufferUpdate = VncClientSrc::update_vnc;

  int argc = 2;
  char* server_address = const_cast<char*>(vnc_server_address_.c_str());
  char* argv[] = {(char*)"switcher", server_address};
  if (!rfbInitClient(rfb_client_, &argc, argv)) return false;

  vnc_continue_update_ = true;
  vnc_update_thread_ = thread([&]() {
    while (vnc_continue_update_) {
      int i = WaitForMessage(rfb_client_, 10000);
      if (i < 0) return;
      if (i > 0) {
        if (!HandleRFBServerMessage(rfb_client_)) return;
        if (vnc_writer_) {
          vnc_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(rfb_client_->frameBuffer,
                                                                   framebuffer_size_);
          vnc_writer_->bytes_written(framebuffer_size_);
        }
      }
    }
  });

  return true;
}

bool VncClientSrc::stop() {
  vnc_continue_update_ = false;
  if (vnc_update_thread_.joinable()) vnc_update_thread_.join();

  return true;
}

bool VncClientSrc::connect(string shmdata_socket_path) {
  unique_lock<mutex> connectLock(connect_mutex_);

  int shmreader_id = shmreader_id_;
  shmreader_id_++;

  auto reader = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, size_t size) {
        unique_lock<mutex> lock(mutex_);

        if (rfb_client_ == nullptr) return;

        auto typeIt = shmdata_readers_caps_.find(shmreader_id);
        if (typeIt == shmdata_readers_caps_.end()) return;

        auto type = typeIt->second;

        if (type == string(VNC_MOUSE_EVENTS_CAPS)) {
          if (size < 3 * sizeof(std::uint32_t)) return;

          // Position is stored in integer, and correspond to the position in
          // [0, 1] multiplied by 100000.
          int xPos = (static_cast<float>(static_cast<std::uint32_t*>(data)[0]) / 100000.f) *
                     static_cast<float>(rfb_client_->width);
          int yPos = (static_cast<float>(static_cast<std::uint32_t*>(data)[1]) / 100000.f) *
                     static_cast<float>(rfb_client_->height);
          int buttons = static_cast<std::uint32_t*>(data)[2];
          SendPointerEvent(rfb_client_, xPos, yPos, buttons);
        } else if (type == string(VNC_KEYBOARD_EVENTS_CAPS)) {
          if (size < 2 * sizeof(std::uint32_t)) return;
          unsigned int key = static_cast<std::uint32_t*>(data)[0];
          bool down = static_cast<std::uint32_t*>(data)[1];
          SendKeyEvent(rfb_client_, key, down);
        }
      },
      [=](string caps) {
        unique_lock<mutex> lock(mutex_);
        shmdata_readers_caps_[shmreader_id] = caps;
      });

  events_readers_[shmdata_socket_path] = std::move(reader);

  return true;
}

bool VncClientSrc::disconnect(string shmdata_socket_path) {
  unique_lock<mutex> lock(mutex_);
  auto shmdataIt = events_readers_.find(shmdata_socket_path);
  if (shmdataIt != events_readers_.end()) {
    events_readers_.erase(shmdataIt);
    return true;
  } else {
    return false;
  }
}

bool VncClientSrc::disconnect_all() {
  events_readers_.clear();
  return true;
}

bool VncClientSrc::can_sink_caps(string caps) {
  return (caps == string(VNC_MOUSE_EVENTS_CAPS)) || (caps == string(VNC_KEYBOARD_EVENTS_CAPS));
}

rfbBool VncClientSrc::resize_vnc(rfbClient* client) {
  auto width = client->width;
  auto height = client->height;
  auto depth = client->format.bitsPerPixel;

  auto that = static_cast<VncClientSrc*>(
      rfbClientGetClientData(client, (void*)(&VncClientSrc::resize_vnc)));
  if (!that) return FALSE;
  that->framebuffer_.resize(width * height * depth / 8);

  client->updateRect.x = 0;
  client->updateRect.y = 0;
  client->updateRect.w = width;
  client->updateRect.h = height;

  if (that->capture_truecolor_) {
    client->format.redShift = 0;
    client->format.redMax = 255;
    client->format.greenShift = 8;
    client->format.greenMax = 255;
    client->format.blueShift = 16;
    client->format.blueMax = 255;
  } else {
    client->format.redShift = 11;
    client->format.redMax = 31;
    client->format.greenShift = 5;
    client->format.greenMax = 63;
    client->format.blueShift = 0;
    client->format.blueMax = 31;
  }

  client->frameBuffer = that->framebuffer_.data();
  SetFormatAndEncodings(client);

  return TRUE;
}

void VncClientSrc::update_vnc(rfbClient* client, int, int, int, int) {
  auto that = static_cast<VncClientSrc*>(
      rfbClientGetClientData(client, (void*)(&VncClientSrc::resize_vnc)));

  auto width = client->width;
  auto height = client->height;
  auto depth = client->format.bitsPerPixel;

  that->framebuffer_size_ = width * height * depth / 8;
  if (!that->vnc_writer_ ||
      that->framebuffer_size_ > that->vnc_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>() ||
      that->previous_truecolor_state_ != that->capture_truecolor_) {
    auto data_type = string();
    if (that->capture_truecolor_)
      data_type = "video/x-raw,format=(string)RGBA,width=(int)" + to_string(width) +
                  ",height=(int)" + to_string(height) + ",framerate=30/1";
    else
      data_type = "video/x-raw,format=(string)RGB16,width=(int)" + to_string(width) +
                  ",height=(int)" + to_string(height) + ",framerate=30/1";

    that->previous_truecolor_state_ = that->capture_truecolor_;

    that->vnc_writer_.reset();
    that->vnc_writer_ = std::make_unique<ShmdataWriter>(
        that, that->make_file_name("vnc"), that->framebuffer_size_, data_type);
    if (!that->vnc_writer_) {
      return;
    }
  }
}
}
