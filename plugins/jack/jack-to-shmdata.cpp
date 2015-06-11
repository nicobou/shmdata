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

#include <gst/gst.h>
#include "switcher/std2.hpp"
#include "./jack-to-shmdata.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    JackToShmdata,
    "jacksrc",
    "Jack Audio Device",
    "audio",
    "writer",
    "get audio from jack",
    "LGPL",
    "Nicolas Bouillot");

JackToShmdata::JackToShmdata(const std::string &name):
    custom_props_(std::make_shared<CustomPropertyHelper>()),
    client_name_(name),
    jack_client_(name.c_str()){
  if (jack_client_) {
    jack_client_.set_jack_process_callback(&JackToShmdata::jack_process, this);
    jack_client_.set_on_xrun_callback([this](uint n){on_xrun(n);});
  }
}

bool JackToShmdata::init() {
  if (!jack_client_) {
    g_warning("JackClient cannot be instancied");
    return false;
  }
  init_startable(this);
  num_channels_spec_ =
      custom_props_->make_int_property("channels",
                                       "number of channels",
                                       1,
                                       128,
                                       num_channels_,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       JackToShmdata::set_num_channels,
                                       JackToShmdata::get_num_channels,
                                       this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            num_channels_spec_, "channels", "Channels");
  connect_physical_port_prop_ =
      custom_props_->make_boolean_property("connect",
                                           "automatically connect to physical ports",
                                           (gboolean) TRUE,
                                           (GParamFlags)G_PARAM_READWRITE,
                                           JackToShmdata::set_connect_phys,
                                           JackToShmdata::get_connect_phys,
                                           this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            connect_physical_port_prop_,
                            "connect",
                            "automatically connect to physical ports");
  client_name_spec_ =
      custom_props_->make_string_property("jack-client-name",
                                          "the jack client name",
                                          client_name_.c_str(),
                                          (GParamFlags) G_PARAM_READWRITE,
                                          JackToShmdata::set_client_name,
                                          JackToShmdata::get_client_name,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            client_name_spec_,
                            "client-name", "Client Name");
  return true;
}

bool JackToShmdata::start() {
  buf_.resize(num_channels_ * jack_client_.get_buffer_size()); 
  std::string data_type("audio/x-raw, "
                        "format=(string)F32LE, "
                        "layout=(string)interleaved, "
                        "rate=" + std::to_string(jack_client_.get_sample_rate()) + ", "
                        "channels=" + std::to_string(num_channels_));
  
  std::string shmpath = make_file_name("audio");
  shm_ = std2::make_unique<ShmdataWriter>(this,
                                          shmpath,
                                          buf_.size() * sizeof(jack_sample_t),
                                          data_type);
  if(!shm_.get()) {
    g_warning("JackToShmdata failed to start");
    shm_.reset(nullptr);
    return false;
  }
  { std::unique_lock<std::mutex> lock(input_ports_mutex_);
    for (unsigned int i = 0; i < num_channels_; ++i)
      input_ports_.emplace_back(jack_client_, i, false);
  }
  disable_property("channels");
  disable_property("client-name");
  disable_property("connect");
  return true;
}

bool JackToShmdata::stop() {
  { std::unique_lock<std::mutex> lock(input_ports_mutex_);
    input_ports_.clear();
  }
  shm_.reset(nullptr);
  enable_property("channels");
  enable_property("client-name");
  enable_property("connect");
  return true;
}

void JackToShmdata::set_num_channels(const gint value, void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  context->num_channels_ = value;
  GObjectWrapper::notify_property_changed(context->gobject_->get_gobject(),
                                          context->num_channels_spec_);
}

gint JackToShmdata::get_num_channels(void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  return context->num_channels_;
}

void JackToShmdata::set_client_name(const gchar *value, void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  context->client_name_ = value;
  context->custom_props_->
      notify_property_changed(context->client_name_spec_);
}

const gchar *JackToShmdata::get_client_name(void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  return context->client_name_.c_str();
}

gboolean JackToShmdata::get_connect_phys(void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  if (!context->connect_phys_)
    return FALSE;
  return TRUE;
}

void JackToShmdata::set_connect_phys(gboolean connect, void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  if (connect)
    context->connect_phys_ = true;
  else
    context->connect_phys_ = false;
  context->custom_props_->notify_property_changed(context->connect_physical_port_prop_);
}

int JackToShmdata::jack_process (jack_nframes_t nframes, void *arg){
  auto context = static_cast<JackToShmdata *>(arg);
  { std::unique_lock<std::mutex> lock(context->input_ports_mutex_, std::defer_lock);
    if (lock.try_lock()) {
      std::size_t num_chan = context->input_ports_.size();
      if (0 == num_chan)
        return 0;
      std::vector<jack_sample_t *> jack_bufs;
      for (unsigned int i = 0; i < num_chan; ++i)
        jack_bufs.emplace_back(
            (jack_sample_t *)jack_port_get_buffer(
                context->input_ports_[i].get_raw(), nframes));
      std::size_t index = 0;
      for(unsigned int j = 0; j < nframes; ++j) {
        for (unsigned int i = 0; i < num_chan; ++i){
          context->buf_[index] = jack_bufs[i][j];
          ++index;
        }
      }
      size_t size = nframes * num_chan * sizeof(jack_sample_t);
      context->shm_->writer(&shmdata::Writer::copy_to_shm, context->buf_.data(), size);
      context->shm_->bytes_written(size);
    }  // locked
  }  // releasing lock
  return 0;
}

void JackToShmdata::on_xrun(uint num_of_missed_samples) {
  g_warning ("jack xrun (delay of %u samples)", num_of_missed_samples);
}

}  // namespace swittcher
