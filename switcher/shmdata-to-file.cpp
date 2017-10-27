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

#include <glib/gstdio.h>

#include "./gst-utils.hpp"
#include "./scope-exit.hpp"
#include "./shmdata-to-file.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataToFile,
                                     "shmtofile",
                                     "Shmdata Recorder",
                                     "file",
                                     "reader",
                                     "record shmdata(s) to file(s)",
                                     "LGPL",
                                     "Nicolas Bouillot, Emmanuel Durand");

ShmdataToFile::ShmdataToFile(QuiddityConfiguration&&)
    : custom_prop_(std::make_shared<CustomPropertyHelper>()) {}

ShmdataToFile::~ShmdataToFile() {
  // clean_recorders();
}

bool ShmdataToFile::init_gpipe() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&ShmdataToFile::connect, this, std::placeholders::_1),
                         nullptr,
                         std::bind(&ShmdataToFile::disconnect_all, this),
                         std::bind(&ShmdataToFile::can_sink_caps, this, std::placeholders::_1),
                         8);  // Temporary maximum number

  output_prefix_param_ = custom_prop_->make_string_property("filename_prefix",
                                                            "Prefix to add to the file names",
                                                            output_prefix_.c_str(),
                                                            (GParamFlags)G_PARAM_READWRITE,
                                                            ShmdataToFile::set_output_prefix,
                                                            ShmdataToFile::get_output_prefix,
                                                            this);
  install_property_by_pspec(custom_prop_->get_gobject(),
                            output_prefix_param_,
                            "filename_prefix",
                            "Prefix to add to the file names");

  return true;
}

bool ShmdataToFile::connect(std::string shmdata_socket_path) {
  if (file_names_.find(shmdata_socket_path) != file_names_.end()) {
    warning("ShmdataToFile::connect: % is already added", shmdata_socket_path);
    return false;
  }

  file_names_[shmdata_socket_path] =
      output_prefix_ + shmdata_socket_path.substr(shmdata_socket_path.rfind("/") + 1);

  return true;
}

bool

ShmdataToFile::disconnect_all() {
  if (is_started()) return false;
  file_names_.clear();

  return true;
}

bool ShmdataToFile::start() {
  if (is_started()) return false;

  make_recorders();
  return true;
}

bool ShmdataToFile::stop() {
  if (!is_started()) return false;

  clean_recorders();
  return true;
}

const gchar* ShmdataToFile::get_output_prefix(void* user_data) {
  ShmdataToFile* ctx = static_cast<ShmdataToFile*>(user_data);
  return ctx->output_prefix_.c_str();
}

void ShmdataToFile::set_output_prefix(const gchar* prefix, void* user_data) {
  ShmdataToFile* ctx = static_cast<ShmdataToFile*>(user_data);
  if (prefix != nullptr) ctx->output_prefix_ = prefix;
}

bool ShmdataToFile::can_sink_caps(std::string /*unused*/) {
  return true;  // We can record anything!
}

bool ShmdataToFile::make_recorders() {
  for (auto& it : file_names_) {
    // FIXME check file
    GError* error = nullptr;
    gchar* pipe = g_strdup_printf("gdppay ! filesink location=%s", it.second.c_str());
    On_scope_exit { g_free(pipe); };

    GstElement* recorder_bin = gst_parse_bin_from_description(pipe, TRUE, &error);
    if (error != nullptr) {
      warning("%", error->message);
      g_error_free(error);
      return false;
    }

    g_object_set(G_OBJECT(recorder_bin), "async-handling", TRUE, nullptr);
    gst_bin_add(GST_BIN(get_bin()), recorder_bin);

    ShmdataReader::ptr reader = std::make_shared<ShmdataReader>();
    reader->set_path(it.first.c_str());
    reader->set_bin(get_bin());
    reader->set_g_main_context(get_g_main_context());
    reader->set_sink_element(recorder_bin);
    reader->start();
    register_shmdata(reader);

    GstUtils::sync_state_with_parent(recorder_bin);
  }
  return true;
}

bool ShmdataToFile::clean_recorders() {
  reset_bin();
  clear_shmdatas();

  return true;
}
}
