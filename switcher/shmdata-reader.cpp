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

#include "./shmdata-reader.hpp"
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"

namespace switcher {
ShmdataReader::ShmdataReader():
    reader_(shmdata_base_reader_new()),
    funnel_("funnel"),
    json_description_(std::make_shared<JSONBuilder>()) {
}

ShmdataReader::~ShmdataReader() {
  stop();
}

std::string ShmdataReader::get_path() const {
  return path_;
}

void ShmdataReader::set_path(const char *absolute_path) {
  if (nullptr == absolute_path) {
    g_debug("shmdata path is nullptr");
    return;
  }
  path_ = absolute_path;
  make_json_description();
}

void ShmdataReader::set_bin(GstElement *bin) {
  bin_ = bin;
}

void ShmdataReader::set_g_main_context(GMainContext *context) {
  g_main_context_ = context;
}

void ShmdataReader::set_sink_element(GstElement *sink_element) {
  sink_element_ = sink_element;
}

void ShmdataReader::start() {
  // std::unique_lock<std::mutex> lock (start_mutex_);
  // GstUtils::g_idle_add_full_with_context (g_main_context_,
  //     G_PRIORITY_DEFAULT_IDLE,
  //     start_idle,
  //     this,
  //     nullptr);
  // g_debug ("%s: wait for start idle",
  //      __PRETTY_FUNCTION__);
  // start_cond_.wait (lock);
  // g_debug ("%s: start idle has unlocked",
  //      __PRETTY_FUNCTION__);
  start_idle(this);
}

gboolean ShmdataReader::start_idle(void *user_data) {
  ShmdataReader *context = static_cast<ShmdataReader *>(user_data);
  context->stop();
  g_debug("ShmdataReader::start_idle");
  context->reader_ = shmdata_base_reader_new();
  shmdata_base_reader_set_g_main_context(context->reader_,
                                         context->g_main_context_);
  shmdata_base_reader_set_on_have_type_callback(context->reader_,
                                                ShmdataReader::on_have_type,
                                                context);
  if (context->path_.empty() || context->bin_ == nullptr) {
    g_warning("cannot start the shmdata reader: name or bin or sink element has not bin set");
    return FALSE;
  }
  shmdata_base_reader_set_callback(context->reader_, ShmdataReader::on_first_data, context);
  shmdata_base_reader_install_sync_handler(context->reader_, FALSE);
  shmdata_base_reader_set_bin(context->reader_, context->bin_);
  shmdata_base_reader_start(context->reader_, context->path_.c_str());
  g_debug("ShmdataReader::start_idle done");
  // std::unique_lock<std::mutex> lock (context->start_mutex_);
  // context->start_cond_.notify_all ();
  return FALSE;  // do not repeat
}

void
ShmdataReader::on_have_type(shmdata_base_reader_t *,
                            GstCaps *caps, void *user_data) {
  if (nullptr == user_data || nullptr == caps) {
    g_warning("ShmdataReader::on_have_type cannot save caps");
    return;
  }
  ShmdataReader *context = static_cast<ShmdataReader *>(user_data);
  gchar *string_caps = gst_caps_to_string(caps);
  On_scope_exit {
    if (nullptr != string_caps)
      g_free(string_caps);
  };
  context->set_negociated_caps(std::string(string_caps));
}

void ShmdataReader::stop() {
  g_debug("ShmdataReader::stop");
  shmdata_base_reader_close(reader_);
  reader_ = nullptr;
  UGstElem::renew(funnel_);
}

void
ShmdataReader::set_on_first_data_hook(on_first_data_hook cb,
                                      void *user_data) {
  g_debug("ShmdataReader::set_on_first_data_hook");
  connection_hook_ = cb;
  hook_user_data_ = user_data;
}

void
ShmdataReader::on_first_data(shmdata_base_reader_t *context,
                             void *user_data) {
  ShmdataReader *reader = static_cast<ShmdataReader *>(user_data);
  g_debug(" ShmdataReader::on_first_data");
  if (reader->connection_hook_ != nullptr)    // user want to create the sink_element_
    reader->connection_hook_(reader, reader->hook_user_data_);
  if (nullptr != reader->sink_element_)
    if (!GST_IS_ELEMENT(GST_ELEMENT_PARENT(reader->sink_element_)))
      gst_bin_add(GST_BIN(reader->bin_), reader->sink_element_);
  // else
  //   g_debug ("ShmdataReader::on_first_data: (%s) sink element (%s) has a parent (%s) %d",
  //       reader->get_path ().c_str(),
  //       GST_ELEMENT_NAME (reader->sink_element_),
  //       GST_ELEMENT_NAME(GST_ELEMENT_PARENT (reader->sink_element_)),
  //       GST_IS_ELEMENT(GST_ELEMENT_PARENT (reader->sink_element_)));
  if (!reader->funnel_) {
    g_warning("%s, funnel is empty", __FUNCTION__);
    return;
  }
  gst_bin_add(GST_BIN(reader->bin_), reader->funnel_.get_raw());
  gst_element_link(reader->funnel_.get_raw(), reader->sink_element_);
  GstUtils::sync_state_with_parent(reader->sink_element_);
  GstUtils::sync_state_with_parent(reader->funnel_.get_raw());
  shmdata_base_reader_set_sink(context, reader->funnel_.get_raw());
}

void ShmdataReader::make_json_description() {
  json_description_->reset();
  json_description_->begin_object();
  json_description_->add_string_member("path", path_.c_str());
  json_description_->end_object();
}

JSONBuilder::Node ShmdataReader::get_json_root_node() {
  return json_description_->get_root();
}
}
