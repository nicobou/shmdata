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

quiddity::Factory::Factory() : logger(spdlog::get("switcher")) {
  abstract_factory_.register_kind<quiddities::AudioTestSource>(
      DocumentationRegistry::get()->get_type_from_kind("AudioTestSource"));
  abstract_factory_.register_kind<quiddities::DummySink>(
      DocumentationRegistry::get()->get_type_from_kind("DummySink"));
  abstract_factory_.register_kind<quiddities::EmptyQuiddity>(
      DocumentationRegistry::get()->get_type_from_kind("EmptyQuiddity"));
  abstract_factory_.register_kind<quiddities::ExternalWriter>(
      DocumentationRegistry::get()->get_type_from_kind("ExternalWriter"));
  abstract_factory_.register_kind<quiddities::FileDecoder>(
      DocumentationRegistry::get()->get_type_from_kind("FileDecoder"));
  abstract_factory_.register_kind<quiddities::GstVideoConverter>(
      DocumentationRegistry::get()->get_type_from_kind("GstVideoConverter"));
  abstract_factory_.register_kind<quiddities::GstVideoEncoder>(
      DocumentationRegistry::get()->get_type_from_kind("GstVideoEncoder"));
  abstract_factory_.register_kind<quiddities::GstAudioEncoder>(
      DocumentationRegistry::get()->get_type_from_kind("GstAudioEncoder"));
  abstract_factory_.register_kind<quiddities::GstDecodebin>(
      DocumentationRegistry::get()->get_type_from_kind("GstDecodebin"));
  abstract_factory_.register_kind<quiddities::HTTPSDPDec>(
      DocumentationRegistry::get()->get_type_from_kind("HTTPSDPDec"));
  abstract_factory_.register_kind<quiddities::ShmDelay>(
      DocumentationRegistry::get()->get_type_from_kind("ShmDelay"));
  abstract_factory_.register_kind<quiddities::Timelapse>(
      DocumentationRegistry::get()->get_type_from_kind("Timelapse"));
  abstract_factory_.register_kind<quiddities::Uridecodebin>(
      DocumentationRegistry::get()->get_type_from_kind("Uridecodebin"));
  abstract_factory_.register_kind<quiddities::VideoTestSource>(
      DocumentationRegistry::get()->get_type_from_kind("VideoTestSource"));
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
      LOGGER_DEBUG(this->logger, "loading module {}", absolute_path);
      load_plugin(absolute_path);
    }
    g_free(absolute_path);
    g_object_unref(descend);
    info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  }
  error = nullptr;
  res = g_file_enumerator_close(enumerator, nullptr, &error);
  if (res != TRUE) LOGGER_DEBUG(this->logger, "scanning dir: file enumerator not properly closed");
  if (error != nullptr) LOGGER_DEBUG(this->logger, "scanning dir: error not nullptr");
  g_object_unref(dir);

  plugin_dirs_.emplace_back(directory_path);
  return true;
}

std::string quiddity::Factory::get_default_plugin_dir() const {
  return SWITCHER_DEFAULT_PLUGIN_DIR;
}

std::vector<std::string> quiddity::Factory::get_plugin_dirs() const { return plugin_dirs_; }

InfoTree::ptr quiddity::Factory::get_kinds_doc() const {
  auto kinds_str = std::string(".kinds.");
  auto res = InfoTree::make();
  res->graft(kinds_str, InfoTree::make());
  res->tag_as_array(kinds_str, true);
  for (auto& doc : DocumentationRegistry::get()->get_docs()) {
    auto kind = doc.first;
    auto kind_doc = doc.second;
    res->graft(kinds_str + kind, InfoTree::make());
    auto subtree = res->get_tree(kinds_str + kind);
    subtree->graft(".kind", InfoTree::make(kind));
    subtree->graft(".name", InfoTree::make(kind_doc.get_long_name()));
    subtree->graft(".description", InfoTree::make(kind_doc.get_description()));
    subtree->graft(".license", InfoTree::make(kind_doc.get_license()));
    subtree->graft(".author", InfoTree::make(kind_doc.get_author()));
  }
  return res;
}

bool quiddity::Factory::exists(const std::string& kind) const {
  return abstract_factory_.key_exists(kind);
}

bool quiddity::Factory::load_plugin(const std::string& filename) {
  auto plugin = std::make_unique<PluginLoader>(filename);
  if (!*plugin.get()) {
    LOGGER_WARN(this->logger, plugin->msg());
    return false;
  }
  std::string kind = plugin->get_kind();
  // ignore already loaded plugin
  if (plugins_.end() != plugins_.find(kind)) {
    LOGGER_DEBUG(this->logger, "ignoring already loaded plugin ({} from file {})", kind, filename);
    return false;
  }

  abstract_factory_.register_kind_with_custom_factory(kind, plugin->create_, plugin->destroy_);
  plugins_.emplace(kind, std::move(plugin));
  return true;
}

Quiddity::ptr quiddity::Factory::create(const std::string& kind, quiddity::Config&& config) {
  return abstract_factory_.create(kind, std::forward<quiddity::Config>(config));
}

std::vector<std::string> quiddity::Factory::get_kinds() const {
  return abstract_factory_.get_keys();
}

void quiddity::Factory::register_kind_with_custom_factory(
    const std::string& kind,
    Quiddity* (*custom_create)(quiddity::Config&&),
    void (*custom_destroy)(Quiddity*)) {
  abstract_factory_.register_kind_with_custom_factory(kind, custom_create, custom_destroy);
}

}  // namespace quiddity
}  // namespace switcher
