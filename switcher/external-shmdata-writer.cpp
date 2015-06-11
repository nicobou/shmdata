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

#include "switcher/gst-utils.hpp"
#include "switcher/std2.hpp"
#include "./external-shmdata-writer.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    ExternalShmdataWriter,
    "extshmsrc",
    "Raw Shmdata",
    "other",
    "writer",
    "import an external shmdata writer",
    "LGPL",
    "Nicolas Bouillot");

ExternalShmdataWriter::ExternalShmdataWriter(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

bool ExternalShmdataWriter::init() {
  shmdata_path_spec_ =
      custom_props_->make_string_property("shmdata-path",
                                          "Path Of The Shmdata The Include",
                                          "",
                                          (GParamFlags) G_PARAM_READWRITE,
                                          ExternalShmdataWriter::set_shmdata_path,
                                          ExternalShmdataWriter::get_shmdata_path,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            shmdata_path_spec_, "shmdata-path",
                            "Shmdata Path");
  return true;
}

void
ExternalShmdataWriter::set_shmdata_path(const gchar *value, void *user_data)
{
  ExternalShmdataWriter *context = static_cast<ExternalShmdataWriter *>(user_data);
  context->shmdata_path_ = value;
  context->shm_ = std2::make_unique<ShmdataFollower>(context,
                                                     context->shmdata_path_,
                                                     nullptr,
                                                     nullptr,
                                                     nullptr,
                                                     ".shmdata.writer.");
  context->custom_props_->notify_property_changed(context->shmdata_path_spec_);
}

const gchar *ExternalShmdataWriter::get_shmdata_path(void *user_data) {
  ExternalShmdataWriter *context = static_cast<ExternalShmdataWriter *>(user_data);
  if (context->shmdata_path_.empty())
    return "";
  return context->shmdata_path_.c_str();
}

}  // namespace switcher
