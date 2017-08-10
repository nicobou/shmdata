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

// removing shmdata
#include <gio/gio.h>

#include <fstream>
#include <regex>
#include "./bundle-description-parser.hpp"
#include "./bundle.hpp"

#include "./information-tree-json.hpp"
#include "./scope-exit.hpp"

// the quiddities to manage (line sorted)
#include "./audio-test-source.hpp"
#include "./dummy-sink.hpp"
#include "./empty-quiddity.hpp"
#include "./external-shmdata-writer.hpp"
#include "./gst-audio-encoder.hpp"
#include "./gst-decodebin.hpp"
#include "./gst-video-converter.hpp"
#include "./gst-video-encoder.hpp"
#include "./http-sdp-dec.hpp"
#include "./property-mapper.hpp"
#include "./rtp-session.hpp"
#include "./shm-delay.hpp"
#include "./timelapse.hpp"
#include "./uridecodebin.hpp"
#include "./video-test-source.hpp"

namespace switcher {

QuiddityContainer::ptr QuiddityContainer::make_container(Switcher* switcher,
                                                         BaseLogger* log,
                                                         const std::string& name) {
  QuiddityContainer::ptr container(new QuiddityContainer(name, log));
  container->me_ = container;
  container->switcher_ = switcher;
  return container;
}

QuiddityContainer::QuiddityContainer(const std::string& name, BaseLogger* log)
    : Logged(log), name_(name) {
  remove_shmdata_sockets();
  register_classes();
  make_classes_doc();
}

void QuiddityContainer::release_g_error(GError* error) {
  if (nullptr != error) {
    debug("GError message: %", std::string(error->message));
    g_error_free(error);
  }
}

void QuiddityContainer::remove_shmdata_sockets() {
  std::string dir = Quiddity::get_socket_dir();
  GFile* shmdata_dir = g_file_new_for_commandline_arg(dir.c_str());
  On_scope_exit { g_object_unref(shmdata_dir); };

  gchar* shmdata_prefix =
      g_strconcat(Quiddity::get_socket_name_prefix().c_str(), name_.c_str(), "_", nullptr);
  On_scope_exit { g_free(shmdata_prefix); };

  GError* error = nullptr;
  GFileEnumerator* enumerator = g_file_enumerate_children(
      shmdata_dir, "*", G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS, nullptr, &error);
  release_g_error(error);
  if (nullptr == enumerator) return;
  On_scope_exit {
    g_object_unref(enumerator);
  };
  {
    GError* error = nullptr;
    GFileInfo* info = g_file_enumerator_next_file(enumerator, nullptr, &error);
    release_g_error(error);
    while (info) {
      GFile* descend = g_file_get_child(shmdata_dir, g_file_info_get_name(info));
      On_scope_exit { g_object_unref(descend); };
      char* relative_path = g_file_get_relative_path(shmdata_dir, descend);
      On_scope_exit { g_free(relative_path); };
      if (g_str_has_prefix(relative_path, shmdata_prefix)) {
        gchar* tmp = g_file_get_path(descend);
        debug("deleting file %", tmp);
        if (nullptr != tmp) g_free(tmp);
        if (!g_file_delete(descend, nullptr, &error)) {
          gchar* tmp = g_file_get_path(descend);
          warning("file % cannot be deleted", tmp);
          g_free(tmp);
        }
        On_scope_exit { release_g_error(error); };
      }
      g_object_unref(info);
      info = g_file_enumerator_next_file(enumerator, nullptr, &error);
      release_g_error(error);
    }
  }
}

std::string QuiddityContainer::get_name() const { return name_; }

void QuiddityContainer::register_classes() {
  // registering quiddities
  abstract_factory_.register_class<AudioTestSource>(
      DocumentationRegistry::get()->get_type_from_class_name("AudioTestSource"));
  abstract_factory_.register_class<DummySink>(
      DocumentationRegistry::get()->get_type_from_class_name("DummySink"));
  abstract_factory_.register_class<EmptyQuiddity>(
      DocumentationRegistry::get()->get_type_from_class_name("EmptyQuiddity"));
  abstract_factory_.register_class<ExternalShmdataWriter>(
      DocumentationRegistry::get()->get_type_from_class_name("ExternalShmdataWriter"));
  abstract_factory_.register_class<GstVideoConverter>(
      DocumentationRegistry::get()->get_type_from_class_name("GstVideoConverter"));
  abstract_factory_.register_class<GstVideoEncoder>(
      DocumentationRegistry::get()->get_type_from_class_name("GstVideoEncoder"));
  abstract_factory_.register_class<GstAudioEncoder>(
      DocumentationRegistry::get()->get_type_from_class_name("GstAudioEncoder"));
  abstract_factory_.register_class<GstDecodebin>(
      DocumentationRegistry::get()->get_type_from_class_name("GstDecodebin"));
  abstract_factory_.register_class<HTTPSDPDec>(
      DocumentationRegistry::get()->get_type_from_class_name("HTTPSDPDec"));
  abstract_factory_.register_class<PropertyMapper>(
      DocumentationRegistry::get()->get_type_from_class_name("PropertyMapper"));
  abstract_factory_.register_class<RtpSession>(
      DocumentationRegistry::get()->get_type_from_class_name("RtpSession"));
  abstract_factory_.register_class<ShmDelay>(
      DocumentationRegistry::get()->get_type_from_class_name("ShmDelay"));
  abstract_factory_.register_class<Timelapse>(
      DocumentationRegistry::get()->get_type_from_class_name("Timelapse"));
  abstract_factory_.register_class<Uridecodebin>(
      DocumentationRegistry::get()->get_type_from_class_name("Uridecodebin"));
  abstract_factory_.register_class<VideoTestSource>(
      DocumentationRegistry::get()->get_type_from_class_name("VideoTestSource"));
}

std::vector<std::string> QuiddityContainer::get_classes() { return abstract_factory_.get_keys(); }

void QuiddityContainer::make_classes_doc() {
  auto classes_str = std::string(".classes.");
  classes_doc_ = InfoTree::make();
  classes_doc_->graft(classes_str, InfoTree::make());
  classes_doc_->tag_as_array(classes_str, true);
  for (auto& doc : DocumentationRegistry::get()->get_docs()) {
    auto class_name = doc.first;
    auto class_doc = doc.second;
    classes_doc_->graft(classes_str + class_name, InfoTree::make());
    auto subtree = classes_doc_->get_tree(classes_str + class_name);
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
}

std::string QuiddityContainer::get_classes_doc() {
  return JSONSerializer::serialize(classes_doc_.get());
}

std::string QuiddityContainer::get_class_doc(const std::string& class_name) {
  auto tree = classes_doc_->get_tree(std::string(".classes.") + class_name);
  return JSONSerializer::serialize(tree.get());
}

bool QuiddityContainer::class_exists(const std::string& class_name) {
  return abstract_factory_.key_exists(class_name);
}

std::string QuiddityContainer::create(const std::string& quiddity_class, bool call_creation_cb) {
  return create(quiddity_class, std::string(), call_creation_cb);
}

std::string QuiddityContainer::create(const std::string& quiddity_class,
                                      const std::string& raw_nick_name,
                                      bool call_creation_cb) {
  // checks before creation
  if (!class_exists(quiddity_class)) {
    warning("cannot create quiddity: class % is unknown", quiddity_class);
    return std::string();
  }
  std::string name;
  if (raw_nick_name.empty()) {
    name = quiddity_class + std::to_string(counters_.get_count(quiddity_class));
    while (quiddities_.end() != quiddities_.find(name))
      name = quiddity_class + std::to_string(counters_.get_count(quiddity_class));
  } else {
    name = Quiddity::string_to_quiddity_name(raw_nick_name);
  }
  auto it = quiddities_.find(name);
  if (quiddities_.end() != it) {
    warning("cannot create a quiddity named %, name is already taken", name);
    return std::string();
  }

  // building configuration for quiddity creation
  InfoTree::ptr tree;
  if (configurations_) {
    tree = configurations_->get_tree(quiddity_class);
    if (!tree->branch_has_data("")) tree = configurations_->get_tree("bundle." + quiddity_class);
  }

  // creation
  Quiddity::ptr quiddity = abstract_factory_.create(
      quiddity_class, QuiddityConfiguration(name, quiddity_class, tree, this, get_log_ptr()));
  if (!quiddity) {
    warning("abstract factory failed to create % (class %)", name, quiddity_class);
    return std::string();
  }

  name = quiddity->get_name();

  if (!(*quiddity.get())) {
    debug("creation of % with name % failed", quiddity_class, quiddity->get_name());
    return std::string();
  }
  quiddities_[name] = quiddity;

  if (call_creation_cb) {
    // We work on a copy in case a callback modifies the map of registered callbacks
    auto tmp_created_cbs = on_created_cbs_;
    for (auto& cb : tmp_created_cbs) {
      cb.second(quiddity->get_name());
      if (on_created_cbs_.empty()) break;  // In case the map gets reset in the callback, e.g bundle
    }
  }
  return name;
}

std::vector<std::string> QuiddityContainer::get_instances() const {
  std::vector<std::string> res;
  for (auto& it : quiddities_) res.push_back(it.first);
  return res;
}

bool QuiddityContainer::has_instance(const std::string& name) const {
  return quiddities_.end() != quiddities_.find(name);
}

std::string QuiddityContainer::get_quiddities_description() {
  auto tree = InfoTree::make();
  tree->graft("quiddities", InfoTree::make());
  tree->tag_as_array("quiddities", true);
  auto subtree = tree->get_tree("quiddities");
  std::vector<std::string> quids = get_instances();
  for (auto& it : quids) {
    std::shared_ptr<Quiddity> quid = get_quiddity(it);
    if (quid) {
      std::string name = quid->get_name();
      subtree->graft(name + ".id", InfoTree::make(name));
      subtree->graft(name + ".class", InfoTree::make(quid->get_type()));
    }
  }
  return JSONSerializer::serialize(tree.get());
}

std::string QuiddityContainer::get_quiddity_description(const std::string& name) {
  auto it = quiddities_.find(name);
  if (quiddities_.end() == it) return JSONSerializer::serialize(InfoTree::make().get());
  auto tree = InfoTree::make();
  tree->graft(".id", InfoTree::make(name));
  tree->graft(".class", InfoTree::make(it->second->get_type()));
  return JSONSerializer::serialize(tree.get());
}

Quiddity::ptr QuiddityContainer::get_quiddity(const std::string& quiddity_name) {
  auto it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == it) {
    debug("quiddity % not found, cannot provide ptr", quiddity_name);
    Quiddity::ptr empty_quiddity_ptr;
    return empty_quiddity_ptr;
  }
  return it->second;
}

bool QuiddityContainer::remove(const std::string& quiddity_name, bool call_removal_cb) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    warning("(%) quiddity % not found for removing", name_, quiddity_name);
    return false;
  }
  quiddities_.erase(quiddity_name);
  if (call_removal_cb) {
    // We work on a copy in case a callback modifies the map of registered callbacks
    auto tmp_removed_cbs_ = on_removed_cbs_;
    for (auto& cb : tmp_removed_cbs_) {
      cb.second(quiddity_name);
      if (on_removed_cbs_.empty()) break;  // In case the map gets reset in the callback, e.g bundle
    }
  }
  return true;
}

bool QuiddityContainer::has_method(const std::string& quiddity_name,
                                   const std::string& method_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    debug("quiddity % not found", quiddity_name);
    return false;
  }
  return q_it->second->has_method(method_name);
}

bool QuiddityContainer::invoke(const std::string& quiddity_name,
                               const std::string& method_name,
                               std::string** return_value,
                               const std::vector<std::string>& args) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    debug("quiddity % not found, cannot invoke", quiddity_name);
    if (return_value != nullptr) *return_value = new std::string();
    return false;
  }
  if (!q_it->second->has_method(method_name)) {
    debug("method % not found, cannot invoke", method_name);
    if (return_value != nullptr) *return_value = new std::string();
    return false;
  }
  return q_it->second->invoke_method(method_name, return_value, args);
}

std::string QuiddityContainer::get_methods_description(const std::string& quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    warning("quiddity % not found, cannot get description of methods", quiddity_name);
    return "{\"error\":\"quiddity not found\"}";
  }
  return q_it->second->get_methods_description();
}

std::string QuiddityContainer::get_method_description(const std::string& quiddity_name,
                                                      const std::string& method_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    warning("quiddity % not found, cannot get description of methods", quiddity_name);
    return "{\"error\":\"quiddity not found\"}";
  }

  return q_it->second->get_method_description(method_name);
}

std::string QuiddityContainer::get_methods_description_by_class(const std::string& class_name) {
  if (!class_exists(class_name)) return "{\"error\":\"class not found\"}";
  const std::string& quid_name = create(class_name, false);
  const std::string& descr = get_methods_description(quid_name);
  remove(quid_name, false);
  return descr;
}

std::string QuiddityContainer::get_method_description_by_class(const std::string& class_name,
                                                               const std::string& method_name) {
  if (!class_exists(class_name)) return "{\"error\":\"class not found\"}";
  const std::string& quid_name = create(class_name, false);
  const std::string& descr = get_method_description(quid_name, method_name);
  remove(quid_name, false);
  return descr;
}

unsigned int QuiddityContainer::register_creation_cb(OnCreateRemoveCb cb) {
  static unsigned int id = 0;
  id %= std::numeric_limits<unsigned int>::max();
  me_.lock();
  on_created_cbs_[++id] = cb;
  return id;
}

unsigned int QuiddityContainer::register_removal_cb(OnCreateRemoveCb cb) {
  static unsigned int id = 0;
  id %= std::numeric_limits<unsigned int>::max();
  me_.lock();
  on_removed_cbs_[++id] = cb;
  return id;
}

void QuiddityContainer::unregister_creation_cb(unsigned int id) {
  me_.lock();
  auto it = on_created_cbs_.find(id);
  if (it != on_created_cbs_.end()) on_created_cbs_.erase(it);
}

void QuiddityContainer::unregister_removal_cb(unsigned int id) {
  me_.lock();
  auto it = on_removed_cbs_.find(id);
  if (it != on_removed_cbs_.end()) on_removed_cbs_.erase(it);
}

void QuiddityContainer::reset_create_remove_cb() {
  me_.lock();
  on_created_cbs_.clear();
  on_removed_cbs_.clear();
}

bool QuiddityContainer::load_plugin(const char* filename) {
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

void QuiddityContainer::close_plugin(const std::string& class_name) {
  abstract_factory_.unregister_class(class_name);
  plugins_.erase(class_name);
}

std::vector<std::string> QuiddityContainer::get_plugin_dirs() const { return plugin_dirs_; }

bool QuiddityContainer::scan_directory_for_plugins(const std::string& directory_path) {
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

  make_classes_doc();
  plugin_dirs_.emplace_back(directory_path);
  return true;
}

bool QuiddityContainer::load_configuration_file(const std::string& file_path) {
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    warning("cannot open % for loading configuration", file_path);
    return false;
  }
  // get file content into a string
  std::string config;
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    warning("file % is empty", file_path);
    return false;
  }
  if (size > kMaxConfigurationFileSize) {
    warning("file % is too large, max is % bytes",
            file_path,
            std::to_string(kMaxConfigurationFileSize));
    return false;
  }
  config.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  config.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  // building the tree
  auto tree = JSONSerializer::deserialize(config);
  if (!tree) {
    warning("configuration tree cannot be constructed from file %", file_path);
    return false;
  }
  // writing new configuration
  configurations_ = tree;

  // registering bundle(s) as creatable class
  auto quid_types = abstract_factory_.get_keys();
  for (auto& it : configurations_->get_child_keys("bundle")) {
    if (std::string::npos != it.find('_')) {
      warning("underscores are not allowed for quiddity types (bundle name %)", it);
      continue;
    }
    std::string long_name = configurations_->branch_get_value("bundle." + it + ".doc.long_name");
    std::string category = configurations_->branch_get_value("bundle." + it + ".doc.category");
    std::string tags = configurations_->branch_get_value("bundle." + it + ".doc.tags");
    std::string description =
        configurations_->branch_get_value("bundle." + it + ".doc.description");
    std::string pipeline = configurations_->branch_get_value("bundle." + it + ".pipeline");
    std::string is_missing;
    if (long_name.empty()) is_missing = "long_name";
    if (category.empty()) is_missing = "category";
    if (tags.empty()) is_missing = "tags";
    if (description.empty()) is_missing = "description";
    if (pipeline.empty()) is_missing = "pipeline";
    if (!is_missing.empty()) {
      warning("% : % field is missing, cannot create new quiddity type", it, is_missing);
      continue;
    }
    // check if the pipeline description is correct
    auto spec = bundle::DescriptionParser(pipeline, quid_types);
    if (!spec) {
      warning("% : error parsing the pipeline (%)", it, spec.get_parsing_error());
      continue;
    }
    // ok, bundle can be added
    DocumentationRegistry::get()->register_doc(
        it, QuiddityDocumentation(long_name, it, category, tags, description, "n/a", "n/a"));

    abstract_factory_.register_class_with_custom_factory(it, &bundle::create, &bundle::destroy);
    // making the new bundle type available for next bundle definition:
    quid_types.push_back(it);
  }
  make_classes_doc();
  return true;
}

}  // namespace switcher
