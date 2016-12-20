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
#include "./logger.hpp"
#include "./property-mapper.hpp"
#include "./rtp-session.hpp"
#include "./timelapse.hpp"
#include "./uridecodebin.hpp"
#include "./video-test-source.hpp"

namespace switcher {
QuiddityManager_Impl::ptr QuiddityManager_Impl::make_manager(QuiddityManager* root_manager,
                                                             const std::string& name) {
  QuiddityManager_Impl::ptr manager(new QuiddityManager_Impl(name));
  manager->me_ = manager;
  manager->manager_ = root_manager;
  return manager;
}

QuiddityManager_Impl::QuiddityManager_Impl(const std::string& name) : name_(name) {
  remove_shmdata_sockets();
  register_classes();
  make_classes_doc();
}

QuiddityManager_Impl::~QuiddityManager_Impl() {
  Quiddity::ptr logger;
  std::find_if(quiddities_.begin(),
               quiddities_.end(),
               [&logger](const std::pair<std::string, std::shared_ptr<Quiddity>> quid) {
                 if (quid.second->get_documentation()->get_class_name() == "logger") {
                   // We increment the logger ref count so it's not destroyed by clear().
                   logger = quid.second;
                   return true;
                 }
                 return false;
               });
  quiddities_.clear();
}

void QuiddityManager_Impl::release_g_error(GError* error) {
  if (nullptr != error) {
    g_debug("GError message: %s\n", error->message);
    g_error_free(error);
  }
}

void QuiddityManager_Impl::remove_shmdata_sockets() {
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
    // GError *error = nullptr;
    // g_file_enumerator_close(enumerator, nullptr, &error);
    // release_g_error(error);
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
        g_debug("deleting file %s", tmp);
        if (nullptr != tmp) g_free(tmp);
        if (!g_file_delete(descend, nullptr, &error)) {
          gchar* tmp = g_file_get_path(descend);
          g_warning("file %s cannot be deleted", tmp);
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

std::string QuiddityManager_Impl::get_name() const { return name_; }

void QuiddityManager_Impl::register_classes() {
  // registering quiddities
  abstract_factory_.register_class<AudioTestSource>(AudioTestSource::switcher_doc_.get_class_name(),
                                                    &AudioTestSource::switcher_doc_);
  abstract_factory_.register_class<DummySink>(DummySink::switcher_doc_.get_class_name(),
                                              &DummySink::switcher_doc_);
  abstract_factory_.register_class<EmptyQuiddity>(EmptyQuiddity::switcher_doc_.get_class_name(),
                                                  &EmptyQuiddity::switcher_doc_);
  abstract_factory_.register_class<ExternalShmdataWriter>(
      ExternalShmdataWriter::switcher_doc_.get_class_name(), &ExternalShmdataWriter::switcher_doc_);
  abstract_factory_.register_class<GstVideoConverter>(
      GstVideoConverter::switcher_doc_.get_class_name(), &GstVideoConverter::switcher_doc_);
  abstract_factory_.register_class<GstVideoEncoder>(GstVideoEncoder::switcher_doc_.get_class_name(),
                                                    &GstVideoEncoder::switcher_doc_);
  abstract_factory_.register_class<GstAudioEncoder>(GstAudioEncoder::switcher_doc_.get_class_name(),
                                                    &GstAudioEncoder::switcher_doc_);
  abstract_factory_.register_class<GstDecodebin>(GstDecodebin::switcher_doc_.get_class_name(),
                                                 &GstDecodebin::switcher_doc_);
  // abstract_factory_.register_class<GstParseToBinSrc>
  //     (GstParseToBinSrc::switcher_doc_.get_class_name(),
  //      &GstParseToBinSrc::switcher_doc_);
  abstract_factory_.register_class<HTTPSDPDec>(HTTPSDPDec::switcher_doc_.get_class_name(),
                                               &HTTPSDPDec::switcher_doc_);
  abstract_factory_.register_class<Logger>(Logger::switcher_doc_.get_class_name(),
                                           &Logger::switcher_doc_);
  abstract_factory_.register_class<PropertyMapper>(PropertyMapper::switcher_doc_.get_class_name(),
                                                   &PropertyMapper::switcher_doc_);
  abstract_factory_.register_class<RtpSession>(RtpSession::switcher_doc_.get_class_name(),
                                               &RtpSession::switcher_doc_);
  // abstract_factory_.register_class<ShmdataFromGDPFile>
  //     (ShmdataFromGDPFile::switcher_doc_.get_class_name(),
  //      &ShmdataFromGDPFile::switcher_doc_);
  // abstract_factory_.register_class<ShmdataToFile>
  //     (ShmdataToFile::switcher_doc_.get_class_name(),
  //      &ShmdataToFile::switcher_doc_);
  abstract_factory_.register_class<Timelapse>(Timelapse::switcher_doc_.get_class_name(),
                                              &Timelapse::switcher_doc_);
  abstract_factory_.register_class<Uridecodebin>(Uridecodebin::switcher_doc_.get_class_name(),
                                                 &Uridecodebin::switcher_doc_);
  abstract_factory_.register_class<VideoTestSource>(VideoTestSource::switcher_doc_.get_class_name(),
                                                    &VideoTestSource::switcher_doc_);
}

std::vector<std::string> QuiddityManager_Impl::get_classes() {
  return abstract_factory_.get_keys();
}

void QuiddityManager_Impl::make_classes_doc() {
  std::vector<QuiddityDocumentation*> docs = abstract_factory_.get_classes_documentation();
  auto classes_str = std::string(".classes.");
  classes_doc_ = InfoTree::make();
  classes_doc_->graft(classes_str, InfoTree::make());
  classes_doc_->tag_as_array(classes_str, true);
  for (auto& doc : docs) {
    auto class_name = doc->get_class_name();
    classes_doc_->graft(classes_str + class_name, InfoTree::make());
    auto subtree = classes_doc_->get_tree(classes_str + class_name);
    subtree->graft(".class", InfoTree::make(class_name));
    subtree->graft(".name", InfoTree::make(doc->get_long_name()));
    subtree->graft(".category", InfoTree::make(doc->get_category()));
    auto tags = doc->get_tags();
    subtree->graft(".tags", InfoTree::make());
    subtree->tag_as_array(".tags", true);
    for (auto& tag : tags) subtree->graft(".tags." + tag, InfoTree::make(tag));
    subtree->graft(".description", InfoTree::make(doc->get_description()));
    subtree->graft(".license", InfoTree::make(doc->get_license()));
    subtree->graft(".author", InfoTree::make(doc->get_author()));
  }
}

std::string QuiddityManager_Impl::get_classes_doc() {
  return JSONSerializer::serialize(classes_doc_.get());
}

std::string QuiddityManager_Impl::get_class_doc(const std::string& class_name) {
  auto tree = classes_doc_->get_tree(std::string(".classes.") + class_name);
  return JSONSerializer::serialize(tree.get());
}

bool QuiddityManager_Impl::class_exists(const std::string& class_name) {
  return abstract_factory_.key_exists(class_name);
}

bool QuiddityManager_Impl::init_quiddity(Quiddity::ptr quiddity) {
  quiddity->set_manager_impl(me_.lock());
  if (!quiddity->init()) return false;

  quiddities_[quiddity->get_name()] = quiddity;

  for (auto& cb : on_created_cbs_) cb.second(quiddity->get_name());

  return true;
}

std::string QuiddityManager_Impl::create(const std::string& quiddity_class, bool call_creation_cb) {
  return create(quiddity_class, std::string(), call_creation_cb);
}

std::string QuiddityManager_Impl::create(const std::string& quiddity_class,
                                         const std::string& raw_nick_name,
                                         bool call_creation_cb) {
  if (!class_exists(quiddity_class)) {
    g_warning("cannot create quiddity: class %s is unknown", quiddity_class.c_str());
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
    g_warning("cannot create a quiddity named %s, name is already taken", name.c_str());
    return std::string();
  }

  Quiddity::ptr quiddity = abstract_factory_.create(quiddity_class, name);
  if (!quiddity) {
    g_warning(
        "abstract factory failed to create %s (class %s)", name.c_str(), quiddity_class.c_str());
    return std::string();
  }

  auto bundle_doc_it = bundle_docs_.find(quiddity_class);
  if (bundle_doc_it != bundle_docs_.end()) {
    static_cast<Bundle*>(quiddity.get())->set_doc_getter([this, quiddity_class]() {
      return bundle_docs_.find(quiddity_class)->second.get();
    });
    quiddity->set_configuration(configurations_->get_tree("bundle." + quiddity_class));
  } else {
    if (configurations_) {
      auto tree = configurations_->get_tree(quiddity_class);
      if (tree) quiddity->set_configuration(configurations_->get_tree(quiddity_class));
    }
  }

  quiddity->set_name(name);
  name = quiddity->get_name();
  if (!call_creation_cb) {
    if (!quiddity->init()) return "{\"error\":\"cannot init quiddity class\"}";
    quiddities_[name] = quiddity;
  } else {
    if (!init_quiddity(quiddity)) {
      g_debug("initialization of %s with name %s failed",
              quiddity_class.c_str(),
              quiddity->get_name().c_str());
      return std::string();
    }
  }

  return name;
}

std::vector<std::string> QuiddityManager_Impl::get_instances() const {
  std::vector<std::string> res;
  for (auto& it : quiddities_) res.push_back(it.first);
  return res;
}

bool QuiddityManager_Impl::has_instance(const std::string& name) const {
  return quiddities_.end() != quiddities_.find(name);
}

std::string QuiddityManager_Impl::get_quiddities_description() {
  auto tree = InfoTree::make();
  tree->graft("quiddities", InfoTree::make());
  tree->tag_as_array("quiddities", true);
  auto subtree = tree->get_tree("quiddities");
  std::vector<std::string> quids = get_instances();
  for (auto& it : quids) {
    std::shared_ptr<Quiddity> quid = get_quiddity(it);
    auto name = quid->get_name();
    subtree->graft(name + ".id", InfoTree::make(quid->get_name()));
    subtree->graft(name + ".class", InfoTree::make(quid->get_documentation()->get_class_name()));
  }
  return JSONSerializer::serialize(tree.get());
}

std::string QuiddityManager_Impl::get_quiddity_description(const std::string& nick_name) {
  auto it = quiddities_.find(nick_name);
  if (quiddities_.end() == it) return JSONSerializer::serialize(InfoTree::make().get());
  auto tree = InfoTree::make();
  tree->graft(".id", InfoTree::make(nick_name));
  tree->graft(".class", InfoTree::make(it->second->get_documentation()->get_class_name()));
  return JSONSerializer::serialize(tree.get());
}

InfoTree::ptr QuiddityManager_Impl::get_quiddity_description2(const std::string& nick_name) {
  auto res = InfoTree::make();
  auto found = quiddities_.find(nick_name);
  if (quiddities_.end() == found) return res;
  res->graft(std::string("name"), InfoTree::make(nick_name));
  res->graft(std::string("class"),
             InfoTree::make(found->second->get_documentation()->get_class_name()));
  return res;
}

Quiddity::ptr QuiddityManager_Impl::get_quiddity(const std::string& quiddity_name) {
  auto it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == it) {
    g_debug("quiddity %s not found, cannot provide ptr", quiddity_name.c_str());
    Quiddity::ptr empty_quiddity_ptr;
    return empty_quiddity_ptr;
  }
  return it->second;
}

bool QuiddityManager_Impl::remove(const std::string& quiddity_name, bool call_removal_cb) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("(%s) quiddity %s not found for removing", name_.c_str(), quiddity_name.c_str());
    return false;
  }
  for (auto& it : signal_subscribers_) it.second->unsubscribe(q_it->second);
  quiddities_.erase(quiddity_name);
  if (call_removal_cb) {
    for (auto& cb : on_removed_cbs_) cb.second(quiddity_name);
  }
  return true;
}

bool QuiddityManager_Impl::has_method(const std::string& quiddity_name,
                                      const std::string& method_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_debug("quiddity %s not found", quiddity_name.c_str());
    return false;
  }
  return q_it->second->has_method(method_name);
}

bool QuiddityManager_Impl::invoke(const std::string& quiddity_name,
                                  const std::string& method_name,
                                  std::string** return_value,
                                  const std::vector<std::string>& args) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_debug("quiddity %s not found, cannot invoke", quiddity_name.c_str());
    if (return_value != nullptr) *return_value = new std::string();
    return false;
  }
  if (!q_it->second->has_method(method_name)) {
    g_debug("method %s not found, cannot invoke", method_name.c_str());
    if (return_value != nullptr) *return_value = new std::string();
    return false;
  }
  return q_it->second->invoke_method(method_name, return_value, args);
}

std::string QuiddityManager_Impl::get_methods_description(const std::string& quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get description of methods", quiddity_name.c_str());
    return "{\"error\":\"quiddity not found\"}";
  }
  return q_it->second->get_methods_description();
}

std::string QuiddityManager_Impl::get_method_description(const std::string& quiddity_name,
                                                         const std::string& method_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get description of methods", quiddity_name.c_str());
    return "{\"error\":\"quiddity not found\"}";
  }

  return q_it->second->get_method_description(method_name);
}

std::string QuiddityManager_Impl::get_methods_description_by_class(const std::string& class_name) {
  if (!class_exists(class_name)) return "{\"error\":\"class not found\"}";
  const std::string& quid_name = create(class_name, false);
  const std::string& descr = get_methods_description(quid_name);
  remove(quid_name, false);
  return descr;
}

std::string QuiddityManager_Impl::get_method_description_by_class(const std::string& class_name,
                                                                  const std::string& method_name) {
  if (!class_exists(class_name)) return "{\"error\":\"class not found\"}";
  const std::string& quid_name = create(class_name, false);
  const std::string& descr = get_method_description(quid_name, method_name);
  remove(quid_name, false);
  return descr;
}

// *** signals
std::string QuiddityManager_Impl::get_signals_description(const std::string& quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get signals description", quiddity_name.c_str());
    return std::string();
  }
  return q_it->second->get_signals_description();
}

std::string QuiddityManager_Impl::get_signal_description(const std::string& quiddity_name,
                                                         const std::string& signal_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get signal description", quiddity_name.c_str());
    return std::string();
  }
  return q_it->second->get_signal_description(signal_name);
}

std::string QuiddityManager_Impl::get_signals_description_by_class(const std::string& class_name) {
  if (!class_exists(class_name)) return "{\"error\":\"class not found\"}";
  std::string quid_name = create(class_name, false);
  if (quid_name.empty())
    return "{\"error\":\"cannot get signal description because the class "
           "cannot be instanciated\"}";
  std::string descr = get_signals_description(quid_name);
  remove(quid_name, false);
  return descr;
}

std::string QuiddityManager_Impl::get_signal_description_by_class(const std::string& class_name,
                                                                  const std::string& signal_name) {
  if (!class_exists(class_name)) return "{\"error\":\"class not found\"}";
  std::string quid_name = create(class_name, false);
  if (quid_name.empty())
    return "{\"error\":\"cannot get signal because the class cannot be "
           "instanciated\"}";
  std::string descr = get_signal_description(quid_name, signal_name);
  remove(quid_name, false);
  return descr;
}

// higher level subscriber
void QuiddityManager_Impl::mute_signal_subscribers(bool muted) {
  for (auto& it : signal_subscribers_) it.second->mute(muted);
}

bool QuiddityManager_Impl::make_signal_subscriber(const std::string& subscriber_name,
                                                  QuidditySignalSubscriber::OnEmittedCallback cb,
                                                  void* user_data) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() != it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s already exists\n",
              subscriber_name.c_str());
    return false;
  }
  QuidditySignalSubscriber::ptr subscriber = std::make_shared<QuidditySignalSubscriber>();
  subscriber->set_manager_impl(me_.lock());
  subscriber->set_name(subscriber_name.c_str());
  subscriber->set_user_data(user_data);
  subscriber->set_callback(cb);
  signal_subscribers_[subscriber_name] = subscriber;
  return true;
}

bool QuiddityManager_Impl::remove_signal_subscriber(const std::string& subscriber_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s does not exists\n",
              subscriber_name.c_str());
    return false;
  }
  signal_subscribers_.erase(it);
  return true;
}

bool QuiddityManager_Impl::subscribe_signal(const std::string& subscriber_name,
                                            const std::string& quiddity_name,
                                            const std::string& signal_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s does not exists\n",
              subscriber_name.c_str());
    return false;
  }
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot subscribe to signal", quiddity_name.c_str());
    return false;
  }
  return it->second->subscribe(q_it->second, signal_name);
}

bool QuiddityManager_Impl::unsubscribe_signal(const std::string& subscriber_name,
                                              const std::string& quiddity_name,
                                              const std::string& signal_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s does not exists\n",
              subscriber_name.c_str());
    return false;
  }
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot subscribe to signal", quiddity_name.c_str());
    return false;
  }
  return it->second->unsubscribe(q_it->second, signal_name);
}

std::vector<std::string> QuiddityManager_Impl::list_signal_subscribers() const {
  std::vector<std::string> res;
  for (auto& it : signal_subscribers_) res.push_back(it.first);
  return res;
}

std::vector<std::pair<std::string, std::string>> QuiddityManager_Impl::list_subscribed_signals(
    const std::string& subscriber_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s does not exists\n",
              subscriber_name.c_str());
    std::vector<std::pair<std::string, std::string>> empty;
    return empty;
  }
  return it->second->list_subscribed_signals();
}

std::string QuiddityManager_Impl::list_subscribed_signals_json(const std::string& subscriber_name) {
  auto subscribed_sigs = list_subscribed_signals(subscriber_name);
  auto tree = InfoTree::make();
  tree->graft("subscribed signals", InfoTree::make());
  tree->tag_as_array("subscribed signals", true);
  auto subtree = tree->get_tree("subscribed signals");
  for (auto& it : subscribed_sigs) {
    subtree->graft("quiddity", InfoTree::make(it.first));
    subtree->graft("signal", InfoTree::make(it.second));
  }
  return JSONSerializer::serialize(tree.get());
}

unsigned int QuiddityManager_Impl::register_creation_cb(OnCreateRemoveCb cb) {
  static unsigned int id = 0;
  id %= std::numeric_limits<unsigned int>::max();
  on_created_cbs_[++id] = cb;
  return id;
}

unsigned int QuiddityManager_Impl::register_removal_cb(OnCreateRemoveCb cb) {
  static unsigned int id = 0;
  id %= std::numeric_limits<unsigned int>::max();
  on_removed_cbs_[++id] = cb;
  return id;
}

void QuiddityManager_Impl::unregister_creation_cb(unsigned int id) {
  auto it = on_created_cbs_.find(id);
  if (it != on_created_cbs_.end()) on_created_cbs_.erase(it);
}

void QuiddityManager_Impl::unregister_removal_cb(unsigned int id) {
  auto it = on_removed_cbs_.find(id);
  if (it != on_removed_cbs_.end()) on_removed_cbs_.erase(it);
}

void QuiddityManager_Impl::reset_create_remove_cb() {
  on_created_cbs_.clear();
  on_removed_cbs_.clear();
}

bool QuiddityManager_Impl::load_plugin(const char* filename) {
  PluginLoader::ptr plugin = std::make_shared<PluginLoader>();
  if (!plugin->load(filename)) return false;
  std::string class_name = plugin->get_class_name();
  // close the old one if exists
  auto it = plugins_.find(class_name);
  if (plugins_.end() != it) {
    g_debug("closing old plugin for reloading (class: %s)", class_name.c_str());
    close_plugin(class_name);
  }
  abstract_factory_.register_class_with_custom_factory(
      class_name, plugin->get_doc(), plugin->create_, plugin->destroy_);
  plugins_[class_name] = plugin;
  return true;
}

void QuiddityManager_Impl::close_plugin(const std::string& class_name) {
  abstract_factory_.unregister_class(class_name);
  plugins_.erase(class_name);
}

std::vector<std::string> QuiddityManager_Impl::get_plugin_dirs() const { return plugin_dirs_; }

bool QuiddityManager_Impl::scan_directory_for_plugins(const std::string& directory_path) {
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
      g_debug("loading module %s", absolute_path);
      load_plugin(absolute_path);
    }
    g_free(absolute_path);
    g_object_unref(descend);
    info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  }
  error = nullptr;
  res = g_file_enumerator_close(enumerator, nullptr, &error);
  if (res != TRUE) g_debug("scanning dir: file enumerator not properly closed");
  if (error != nullptr) g_debug("scanning dir: error not nullptr");
  g_object_unref(dir);

  make_classes_doc();
  plugin_dirs_.emplace_back(directory_path);
  return true;
}

bool QuiddityManager_Impl::load_configuration_file(const std::string& file_path) {
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    g_warning("cannot open %s for loading configuration", file_path.c_str());
    return false;
  }
  // get file content into a string
  std::string config;
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    g_warning("file %s is empty", file_path.c_str());
    return false;
  }
  if (size > kMaxConfigurationFileSize) {
    g_warning(
        "file %s is too large, max is %d bytes", file_path.c_str(), kMaxConfigurationFileSize);
    return false;
  }
  config.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  config.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  // building the tree
  auto tree = JSONSerializer::deserialize(config);
  if (!tree) {
    g_warning("configuration tree cannot be constructed from file %s", file_path.c_str());
    return false;
  }
  // writing new configuration
  configurations_ = tree;

  // registering bundle(s) as creatable class
  auto quid_types = abstract_factory_.get_keys();
  for (auto& it : configurations_->get_child_keys("bundle")) {
    if (std::string::npos != it.find('_')) {
      g_warning("underscores are not allowed for quiddity types (bundle name %s)", it.c_str());
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
      g_warning("%s : %s field is missing, cannot create new quiddity type",
                it.c_str(),
                is_missing.c_str());
      continue;
    }
    // check if the pipeline description is correct
    auto spec = bundle::DescriptionParser(pipeline, quid_types);
    if (!spec) {
      g_warning(
          "%s : error parsing the pipeline (%s)", it.c_str(), spec.get_parsing_error().c_str());
      continue;
    }
    // ok, bundle can be added
    bundle_docs_.emplace(
        std::make_pair(it,
                       std::make_unique<QuiddityDocumentation>(
                           long_name, it, category, tags, description, "n/a", "n/a")));

    abstract_factory_.register_class_with_custom_factory(
        it, bundle_docs_[it].get(), &bundle::create, &bundle::destroy);
    // making the new bundle type available for next bundle definition:
    quid_types.push_back(it);
  }
  make_classes_doc();
  return true;
}

}  // namespace switcher
