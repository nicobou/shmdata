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

#include "./jack-to-shmdata.hpp"
#include <gst/gst.h>
#include <string.h>
#include "switcher/quiddity-manager-impl.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackToShmdata,
                                     "jacksrc",
                                     "Jack Audio Device",
                                     "audio",
                                     "writer/device",
                                     "get audio from jack",
                                     "LGPL",
                                     "Nicolas Bouillot");

JackToShmdata::JackToShmdata(const std::string& name)
    : client_name_(name),
      jack_client_(name.c_str(),
                   &JackToShmdata::jack_process,
                   this,
                   [this](uint n) { on_xrun(n); },
                   [this](jack_port_t* port) { on_port(port); },
                   [this]() {
                     auto thread = std::thread([this]() {
                       auto manager = manager_impl_.lock();
                       if (!manager) return;
                       if (!manager->remove(get_name()))
                         g_warning("%s did not self destruct after jack shutdown",
                                   get_name().c_str());
                     });
                     thread.detach();
                   }) {
  if (jack_client_) client_name_ = jack_client_.get_name();
}

bool JackToShmdata::init() {
  if (!jack_client_) {
    g_message("ERROR:JackClient cannot be instanciated (is jack server running?)");
    return false;
  }
  init_startable(this);
  num_channels_id_ = pmanage<MPtr(&PContainer::make_int)>("channels",
                                                          [this](const int& val) {
                                                            num_channels_ = val;
                                                            update_port_to_connect();
                                                            return true;
                                                          },
                                                          [this]() { return num_channels_; },
                                                          "Channels",
                                                          "number of channels",
                                                          num_channels_,
                                                          1,
                                                          128);
  client_name_id_ = pmanage<MPtr(&PContainer::make_string)>("jack-client-name",
                                                            [this](const std::string& val) {
                                                              client_name_ = val;
                                                              return true;
                                                            },
                                                            [this]() { return client_name_; },
                                                            "Client Name",
                                                            "The jack client name",
                                                            client_name_);
  auto_connect_id_ = pmanage<MPtr(&PContainer::make_bool)>(
      "auto_connect",
      [this](const bool& val) {
        auto_connect_ = val;
        update_port_to_connect();
        if (auto_connect_) {
          pmanage<MPtr(&PContainer::enable)>(connect_to_id_);
          pmanage<MPtr(&PContainer::enable)>(index_id_);
        } else {
          static const std::string why_disabled =
              "this property is available only when auto connect is enabled";
          pmanage<MPtr(&PContainer::disable)>(connect_to_id_, why_disabled);
          pmanage<MPtr(&PContainer::disable)>(index_id_, why_disabled);
        }
        return true;
      },
      [this]() { return auto_connect_; },
      "Auto Connect",
      "Auto Connect to another client",
      auto_connect_);
  connect_to_id_ = pmanage<MPtr(&PContainer::make_string)>("connect-to",
                                                           [this](const std::string& val) {
                                                             connect_to_ = val;
                                                             update_port_to_connect();
                                                             return true;
                                                           },
                                                           [this]() { return connect_to_; },
                                                           "Connect To",
                                                           "Auto connect to an other client",
                                                           connect_to_);
  index_id_ =
      pmanage<MPtr(&PContainer::make_int)>("index",
                                           [this](const int& val) {
                                             index_ = val;
                                             update_port_to_connect();
                                             return true;
                                           },
                                           [this]() { return index_; },
                                           "Index",
                                           "Start connecting to other client from this index",
                                           num_channels_,
                                           1,
                                           128);
  update_port_to_connect();
  return true;
}

bool JackToShmdata::start() {
  buf_.resize(num_channels_ * jack_client_.get_buffer_size());
  std::string data_type(
      "audio/x-raw, "
      "format=(string)F32LE, "
      "layout=(string)interleaved, "
      "rate=" +
      std::to_string(jack_client_.get_sample_rate()) +
      ", "
      "channels=" +
      std::to_string(num_channels_) + ", channel-mask=(bitmask)0x0");

  std::string shmpath = make_file_name("audio");
  shm_ = std::make_unique<ShmdataWriter>(
      this, shmpath, buf_.size() * sizeof(jack_sample_t), data_type);
  if (!shm_.get()) {
    g_warning("JackToShmdata failed to start");
    shm_.reset(nullptr);
    return false;
  }
  pmanage<MPtr(&PContainer::disable)>(auto_connect_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(num_channels_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(client_name_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(connect_to_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(index_id_, disabledWhenStartedMsg);
  {
    std::lock_guard<std::mutex> lock(input_ports_mutex_);
    for (unsigned int i = 0; i < num_channels_; ++i)
      input_ports_.emplace_back(jack_client_, i + 1, false);
    connect_ports();
  }
  return true;
}

bool JackToShmdata::stop() {
  {
    std::lock_guard<std::mutex> lock(input_ports_mutex_);
    input_ports_.clear();
  }
  shm_.reset(nullptr);
  pmanage<MPtr(&PContainer::enable)>(auto_connect_id_);
  pmanage<MPtr(&PContainer::enable)>(num_channels_id_);
  pmanage<MPtr(&PContainer::enable)>(client_name_id_);
  pmanage<MPtr(&PContainer::enable)>(connect_to_id_);
  pmanage<MPtr(&PContainer::enable)>(index_id_);
  return true;
}

int JackToShmdata::jack_process(jack_nframes_t nframes, void* arg) {
  auto context = static_cast<JackToShmdata*>(arg);
  {
    std::lock_guard<std::mutex> lock(context->port_to_connect_in_jack_process_mutex_);
    for (auto& it : context->port_to_connect_in_jack_process_)
      jack_connect(context->jack_client_.get_raw(), it.first.c_str(), it.second.c_str());
    context->port_to_connect_in_jack_process_.clear();
  }

  {
    std::unique_lock<std::mutex> lock(context->input_ports_mutex_, std::defer_lock);
    if (lock.try_lock()) {
      std::size_t num_chan = context->input_ports_.size();
      if (0 == num_chan) return 0;
      std::vector<jack_sample_t*> jack_bufs;
      for (auto& port : context->input_ports_) {
        jack_sample_t* buf = (jack_sample_t*)jack_port_get_buffer(port.get_raw(), nframes);
        if (!buf) return 0;
        jack_bufs.emplace_back(buf);
      }
      std::size_t index = 0;
      for (unsigned int j = 0; j < nframes; ++j) {
        for (unsigned int i = 0; i < num_chan; ++i) {
          context->buf_[index] = jack_bufs[i][j];
          ++index;
        }
      }
      size_t size = nframes * num_chan * sizeof(jack_sample_t);
      context->shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(context->buf_.data(), size);
      context->shm_->bytes_written(size);
    }  // locked
  }    // releasing lock
  return 0;
}

void JackToShmdata::on_xrun(uint num_of_missed_samples) {
  g_warning("jack xrun (delay of %u samples)", num_of_missed_samples);
}

void JackToShmdata::update_port_to_connect() {
  ports_to_connect_.clear();

  if (!auto_connect_) {
    g_warning("Auto-connect for jack is disabled.");
    return;
  }

  std::lock_guard<std::mutex> lock(port_to_connect_in_jack_process_mutex_);
  for (unsigned int i = index_; i < index_ + num_channels_; ++i)
    ports_to_connect_.emplace_back(connect_to_ + std::to_string(i));
}

void JackToShmdata::connect_ports() {
  if (!auto_connect_) return;

  std::lock_guard<std::mutex> lock(port_to_connect_in_jack_process_mutex_);

  if (ports_to_connect_.size() != input_ports_.size()) {
    g_warning(
        "Port number mismatch in jack to shmdata autoconnect, should not "
        "happen.");
    return;
  }

  for (unsigned int i = 0; i < num_channels_; ++i) {
    jack_connect(jack_client_.get_raw(),
                 ports_to_connect_[i].c_str(),
                 std::string(client_name_ + ":" + input_ports_[i].get_name()).c_str());
  }
}

void JackToShmdata::on_port(jack_port_t* port) {
  int flags = jack_port_flags(port);
  if (!(flags & JackPortIsOutput)) return;
  auto it = std::find(ports_to_connect_.begin(), ports_to_connect_.end(), jack_port_name(port));
  if (ports_to_connect_.end() == it) return;
  {
    std::lock_guard<std::mutex> lock_port_connect(port_to_connect_in_jack_process_mutex_);
    std::lock_guard<std::mutex> lock_input_port(input_ports_mutex_);
    port_to_connect_in_jack_process_.push_back(std::make_pair(
        *it,
        std::string(client_name_ + ":" + input_ports_[it - ports_to_connect_.begin()].get_name())));
  }
}

}  // namespace swittcher
