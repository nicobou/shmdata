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

#include <iostream>

#include <glib/gstdio.h>

#include "./shmdata-to-file.hpp"
#include "./gst-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataToFile,
                                     "Shmdata Recorder",
                                     "file recorder",
                                     "record shmdata(s) to file(s)",
                                     "LGPL",
                                     "shmtofilesink",
                                     "Nicolas Bouillot");

ShmdataToFile::ShmdataToFile():
    custom_prop_(std::make_shared<CustomPropertyHelper>()) {
}

ShmdataToFile::~ShmdataToFile() {
  clean_recorders();
}

bool ShmdataToFile::init_gpipe() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&ShmdataToFile::connect, this, std::placeholders::_1),
                         nullptr,
                         std::bind(&ShmdataToFile::disconnect_all, this),
                         std::bind(&ShmdataToFile::can_sink_caps, this, std::placeholders::_1), 8); // Temporary maximum number

  output_prefix_param_ = custom_prop_->make_string_property("filename_prefix",
                          "Prefix to add to the file names",
                          output_prefix_.c_str(),
                          (GParamFlags) G_PARAM_READWRITE,
                          ShmdataToFile::set_output_prefix,
                          ShmdataToFile::get_output_prefix,
                          this);
  install_property_by_pspec(custom_prop_->get_gobject(),
                          output_prefix_param_,
                          "filename_prefix",
                          "Prefix to add to the file names");

  return true;
}

bool
ShmdataToFile::connect(std::string shmdata_socket_path) {
  if (file_names_.find(shmdata_socket_path) != file_names_.end()) {
    g_warning("ShmdataToFile::connect: %s is already added", shmdata_socket_path.c_str());
    return false;
  }

  file_names_[shmdata_socket_path] = output_prefix_ + shmdata_socket_path.substr(shmdata_socket_path.rfind("/") + 1);

  //ShmdataReader::ptr reader;
  //reader = std::make_shared<ShmdataReader>();
  //reader->set_path(shmdata_socket_path.c_str());
  //reader->set_bin(bin_);
  //reader->set_g_main_context(get_g_main_context());
  //reader->start();

  //shmdata_readers_[shmdata_socket_path] = reader;
  //register_shmdata(shmdata_readers_[shmdata_socket_path]);

  return true;
}

bool

ShmdataToFile::disconnect_all() {
  if (is_started())
    return false;
  file_names_.clear();

  for (auto& reader : shmdata_readers_) {
    reader.second->stop();
  }
  shmdata_readers_.clear();

  return true;
}

bool
ShmdataToFile::start() {
  if (is_started())
    return false;

  make_recorders();
  return true;
}

bool
ShmdataToFile::stop() {
  if (!is_started())
    return false;

  clean_recorders();
  return true;
}


const gchar *
ShmdataToFile::get_output_prefix(void *user_data) {
  ShmdataToFile *ctx = (ShmdataToFile *) user_data;
  return ctx->output_prefix_.c_str();
}

void
ShmdataToFile::set_output_prefix(const gchar *prefix, void *user_data) {
  ShmdataToFile *ctx = (ShmdataToFile *) user_data;
  if (prefix != nullptr)
    ctx->output_prefix_ = prefix;
}

bool
ShmdataToFile::can_sink_caps(std::string /*unused*/) {
  return true; // We can record anything!
}


bool ShmdataToFile::make_recorders() {
  for (auto &it : file_names_) {
    // FIXME check file
    GError *error = nullptr;
    gchar *pipe = g_strdup_printf("gdppay ! filesink location=%s",
                                  it.second.c_str());

    GstElement *recorder_bin = gst_parse_bin_from_description(pipe,
                                                              TRUE,
                                                              &error);
    g_free(pipe);
    if (error != nullptr) {
      g_warning("%s", error->message);
      g_error_free(error);
      return false;
    }
    gst_bin_add(GST_BIN(bin_), recorder_bin);
    // GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent(recorder_bin);

    //shmdata_readers_[it.first]->stop();
    //shmdata_readers_[it.first].reset();
    //unregister_shmdata(it.first);

    ShmdataReader::ptr reader = std::make_shared<ShmdataReader>();
    shmdata_readers_[it.first] = reader;
    reader->set_path(it.first.c_str());
    reader->set_bin(bin_);
    reader->set_g_main_context(get_g_main_context());
    reader->set_sink_element(recorder_bin);
    reader->start();
    register_shmdata(shmdata_readers_[it.first]);

    shmdata_recorders_[it.first] = recorder_bin;
  }
  return true;
}

bool ShmdataToFile::clean_recorders() {
  for (auto &it : shmdata_recorders_) {
    GstUtils::clean_element(it.second);
    unregister_shmdata(it.first);
  }
  shmdata_recorders_.clear();
  return true;
}
}
