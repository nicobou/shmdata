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

#include "./fake-shmdata-writer.hpp"
#include "./gst-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(FakeShmdataWriter,
                                     "Raw Shmdata",
                                     "other",
                                     "add a shmdata from an other software",
                                     "LGPL",
                                     "fakeshmsrc",
                                     "Nicolas Bouillot");

FakeShmdataWriter::FakeShmdataWriter():
    custom_props_(std::make_shared<CustomPropertyHelper>()),
    manager_(QuiddityManager::make_manager("fakeshm-fakesink")) {
  manager_->create("fakesink", "fakesink");
  manager_->make_property_subscriber("sub",
                                     FakeShmdataWriter::caps_cb,
                                     (void *)this);
  manager_->subscribe_property("sub", "fakesink", "caps");
}
    
void FakeShmdataWriter::caps_cb(std::string /*subscriber_name */ ,
                                std::string /*quiddity_name*/,
                                std::string /*property_name*/,
                                std::string value,
                                void *user_data) {
  FakeShmdataWriter *context = static_cast<FakeShmdataWriter *>(user_data);
  context->graft_tree(std::string(".shmdata.writer.") + context->shmdata_path_,
                      data::Tree::make(value));
  g_print("------------------ %s\n", value.c_str());
}

bool FakeShmdataWriter::init() {
  init_startable(this);
  shmdata_path_spec_ =
      custom_props_->make_string_property("shmdata-path",
                                          "Path Of The Shmdata The Include",
                                          "",
                                          (GParamFlags) G_PARAM_READWRITE,
                                          FakeShmdataWriter::set_shmdata_path,
                                          FakeShmdataWriter::get_shmdata_path,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            shmdata_path_spec_, "shmdata-path",
                            "Shmdata Path");

  return true;
}

bool FakeShmdataWriter::add_shmdata_path(std::string name) {
  return true;
}

void
FakeShmdataWriter::set_shmdata_path(const gchar *value, void *user_data)
{
  FakeShmdataWriter *context = static_cast<FakeShmdataWriter *>(user_data);
  context->shmdata_path_ = value;
  context->custom_props_-> notify_property_changed(context->shmdata_path_spec_);
}

const gchar *FakeShmdataWriter::get_shmdata_path(void *user_data) {
  FakeShmdataWriter *context = static_cast<FakeShmdataWriter *>(user_data);
  return context->shmdata_path_.c_str();
}

FakeShmdataWriter::~FakeShmdataWriter() {
}

bool FakeShmdataWriter::start() {
  uninstall_property("shmdata-path");
  if (shmdata_path_ == "none")
    return false;
  manager_->invoke_va("fakesink", "connect", nullptr, shmdata_path_.c_str(), nullptr);
  return true;
}

bool FakeShmdataWriter::stop() {
  // FIXME test this
  // install_property_by_pspec(custom_props_->get_gobject(),
  //                           shmdata_path_spec_,
  //                           "shmdata-path",
  //                           "Shmdata Path");
  return true;
}
}
