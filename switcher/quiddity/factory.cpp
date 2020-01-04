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

#include "./factory.hpp"
#include <gio/gio.h>

// the quiddities to register (line sorted)
#include "../quiddities/audio-test-source.hpp"
#include "../quiddities/dummy-sink.hpp"
#include "../quiddities/empty-quiddity.hpp"
#include "../quiddities/external-shmdata-writer.hpp"
#include "../quiddities/file-decoder.hpp"
#include "../quiddities/gst-audio-encoder.hpp"
#include "../quiddities/gst-decodebin.hpp"
#include "../quiddities/gst-video-converter.hpp"
#include "../quiddities/gst-video-encoder.hpp"
#include "../quiddities/http-sdp-dec.hpp"
#include "../quiddities/shm-delay.hpp"
#include "../quiddities/timelapse.hpp"
#include "../quiddities/uridecodebin.hpp"
#include "../quiddities/video-test-source.hpp"

namespace switcher {
namespace quiddity {

quiddity::Factory::Factory(log::Base* log) : log::Logged(log) {
  abstract_factory_.register_class<quiddities::AudioTestSource>(
      DocumentationRegistry::get()->get_type_from_class_name("AudioTestSource"));
  abstract_factory_.register_class<quiddities::DummySink>(
      DocumentationRegistry::get()->get_type_from_class_name("DummySink"));
  abstract_factory_.register_class<quiddities::EmptyQuiddity>(
      DocumentationRegistry::get()->get_type_from_class_name("EmptyQuiddity"));
  abstract_factory_.register_class<quiddities::ExternalWriter>(
      DocumentationRegistry::get()->get_type_from_class_name("ExternalWriter"));
  abstract_factory_.register_class<quiddities::FileDecoder>(
      DocumentationRegistry::get()->get_type_from_class_name("FileDecoder"));
  abstract_factory_.register_class<quiddities::GstVideoConverter>(
      DocumentationRegistry::get()->get_type_from_class_name("GstVideoConverter"));
  abstract_factory_.register_class<quiddities::GstVideoEncoder>(
      DocumentationRegistry::get()->get_type_from_class_name("GstVideoEncoder"));
  abstract_factory_.register_class<quiddities::GstAudioEncoder>(
      DocumentationRegistry::get()->get_type_from_class_name("GstAudioEncoder"));
  abstract_factory_.register_class<quiddities::GstDecodebin>(
      DocumentationRegistry::get()->get_type_from_class_name("GstDecodebin"));
  abstract_factory_.register_class<quiddities::HTTPSDPDec>(
      DocumentationRegistry::get()->get_type_from_class_name("HTTPSDPDec"));
  abstract_factory_.register_class<quiddities::ShmDelay>(
      DocumentationRegistry::get()->get_type_from_class_name("ShmDelay"));
  abstract_factory_.register_class<quiddities::Timelapse>(
      DocumentationRegistry::get()->get_type_from_class_name("Timelapse"));
  abstract_factory_.register_class<quiddities::Uridecodebin>(
      DocumentationRegistry::get()->get_type_from_class_name("Uridecodebin"));
  abstract_factory_.register_class<quiddities::VideoTestSource>(
      DocumentationRegistry::get()->get_type_from_class_name("VideoTestSource"));
}

bool quiddity::Factory::scan_dir(const std::string& directory_path) {
  GFile* dir = g_file_new_for_commandline_arg(directory_path.c_str());
  gboolean res;
  GError* error;
  GFileEnumerator* enumerator;
  GFileInfo* info;
  error = nullptr;
  enumerator =
      g_file_enumerate_children(dir, "*", G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS, nullptr, &error);
  if (!enumerator) return false;
  error = nullptr;
  info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  while ((info) && (!error)) {
    GFile* descend = g_file_get_child(dir, g_file_info_get_name(info));
    char* absolute_path = g_file_get_path(descend);  // g_file_get_relative_path (dir, descend);
    // trying to load the module
    if (g_str_has_suffix(absolute_path, ".so") || g_str_has_suffix(absolute_path, ".dylib")) {
      debug("loading module %", absolute_path);
      load_plugin(absolute_path);
    }
    g_free(absolute_path);
    g_object_unref(descend);
    info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  }
  error = nullptr;
  res = g_file_enumerator_close(enumerator, nullptr, &error);
  if (res != TRUE) debug("scanning dir: file enumerator not properly closed");
  if (error != nullptr) debug("scanning dir: error not nullptr");
  g_object_unref(dir);

  plugin_dirs_.emplace_back(directory_path);
  return true;
}

std::string quiddity::Factory::get_default_plugin_dir() const {
  return SWITCHER_DEFAULT_PLUGIN_DIR;
}

std::vector<std::string> quiddity::Factory::get_plugin_dirs() const { return plugin_dirs_; }

InfoTree::ptr quiddity::Factory::get_classes_doc() const {
  auto classes_str = std::string(".classes.");
  auto res = InfoTree::make();
  res->graft(classes_str, InfoTree::make());
  res->tag_as_array(classes_str, true);
  for (auto& doc : DocumentationRegistry::get()->get_docs()) {
    auto class_name = doc.first;
    auto class_doc = doc.second;
    res->graft(classes_str + class_name, InfoTree::make());
    auto subtree = res->get_tree(classes_str + class_name);
    subtree->graft(".class", InfoTree::make(class_name));
    subtree->graft(".name", InfoTree::make(class_doc.get_long_name()));
    subtree->graft(".category", InfoTree::make(class_doc.get_category()));
    auto tags = class_doc.get_tags();
    subtree->graft(".tags", InfoTree::make());
    subtree->tag_as_array(".tags", true);
    for (auto& tag : tags) subtree->graft(".tags." + tag, InfoTree::make(tag));
    subtree->graft(".description", InfoTree::make(class_doc.get_description()));
    subtree->graft(".license", InfoTree::make(class_doc.get_license()));
    subtree->graft(".author", InfoTree::make(class_doc.get_author()));
  }
  return res;
}

bool quiddity::Factory::exists(const std::string& class_name) const {
  return abstract_factory_.key_exists(class_name);
}

bool quiddity::Factory::load_plugin(const std::string& filename) {
  PluginLoader::ptr plugin = std::make_shared<PluginLoader>();
  auto loaded = plugin->load(filename);
  if (!loaded) {
    warning("%", loaded.msg());
    return false;
  }
  std::string class_name = plugin->get_class_name();
  // close the old one if exists
  auto it = plugins_.find(class_name);
  if (plugins_.end() != it) {
    debug("closing old plugin for reloading (class: %)", class_name);
    close_plugin(class_name);
  }
  abstract_factory_.register_class_with_custom_factory(
      class_name, plugin->create_, plugin->destroy_);
  plugins_[class_name] = plugin;
  return true;
}

void quiddity::Factory::close_plugin(const std::string& class_name) {
  abstract_factory_.unregister_class(class_name);
  plugins_.erase(class_name);
}

Quiddity::ptr quiddity::Factory::create(const std::string& class_name, quiddity::Config&& config) {
  return abstract_factory_.create(class_name, std::forward<quiddity::Config>(config));
}

std::vector<std::string> quiddity::Factory::get_class_list() const {
  return abstract_factory_.get_keys();
}

void quiddity::Factory::register_class_with_custom_factory(
    const std::string& class_name,
    Quiddity* (*custom_create)(quiddity::Config&&),
    void (*custom_destroy)(Quiddity*)) {
  abstract_factory_.register_class_with_custom_factory(class_name, custom_create, custom_destroy);
}

}  // namespace quiddity
}  // namespace switcher
