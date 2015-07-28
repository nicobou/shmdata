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

#include "./quiddity-documentation.hpp"
#include "./quiddity-manager-impl.hpp"
#include "./quiddity.hpp"
#include "./scope-exit.hpp"
#include "./string-utils.hpp"

// the quiddities to manage (line sorted)
#include "./audio-test-source.hpp"
#include "./create-remove-spy.hpp"
#include "./external-shmdata-writer.hpp"
#include "./gst-audio-encoder.hpp"
#include "./gst-video-encoder.hpp"
#include "./gst-video-converter.hpp"
//#include "./gst-parse-to-bin-src.hpp"
#include "./http-sdp-dec.hpp"
#include "./logger.hpp"
#include "./property-mapper.hpp"
#include "./rtp-session.hpp"
//#include "./shmdata-to-file.hpp"
//#include "./shmdata-from-gdp-file.hpp"
#include "./uridecodebin.hpp"
#include "./string-dictionary.hpp"
#include "./video-test-source.hpp"

namespace switcher {
QuiddityManager_Impl::ptr
QuiddityManager_Impl::make_manager(QuiddityManager *root_manager,
                                   const std::string &name) {
  QuiddityManager_Impl::ptr manager(new QuiddityManager_Impl(name));
  manager->me_ = manager;
  manager->manager_ = root_manager;
  return manager;
}

QuiddityManager_Impl::QuiddityManager_Impl(const std::string &name):
    mainloop_(std::make_shared<GlibMainLoop>()),
    name_(name),
    classes_doc_(std::make_shared<JSONBuilder>()) {
  remove_shmdata_sockets();
  register_classes();
  make_classes_doc();
}

void QuiddityManager_Impl::release_g_error(GError *error) {
  if (nullptr != error) {
    g_debug("GError message: %s\n", error->message);
    g_error_free(error);
  }
}

void QuiddityManager_Impl::remove_shmdata_sockets() {
  std::string dir = Quiddity::get_socket_dir();
  GFile *shmdata_dir =
      g_file_new_for_commandline_arg(dir.c_str());
  On_scope_exit{g_object_unref(shmdata_dir);};

  gchar *shmdata_prefix =
      g_strconcat(Quiddity::get_socket_name_prefix().c_str(),
                  name_.c_str(),
                  "_",
                  nullptr);
  On_scope_exit{g_free(shmdata_prefix);};

  GError *error = nullptr;
  GFileEnumerator *enumerator =
      g_file_enumerate_children(shmdata_dir,
                                "*",
                                G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS,
                                nullptr,
                                &error);
  release_g_error(error);
  if (nullptr == enumerator)
    return;
  On_scope_exit{
    // GError *error = nullptr;
    // g_file_enumerator_close(enumerator, nullptr, &error);
    // release_g_error(error);
    g_object_unref(enumerator);
  };
  {
    GError *error = nullptr;
    GFileInfo *info = g_file_enumerator_next_file(enumerator, nullptr, &error);
    release_g_error(error);
    while (info) {
      GFile *descend = g_file_get_child(shmdata_dir, g_file_info_get_name(info));
      On_scope_exit{g_object_unref(descend);};
        char *relative_path = g_file_get_relative_path(shmdata_dir, descend);
      On_scope_exit{g_free(relative_path);};
      if (g_str_has_prefix(relative_path, shmdata_prefix)) {
        gchar *tmp = g_file_get_path(descend); 
        g_debug("deleting file %s",
                tmp);
        if (nullptr != tmp)
          g_free(tmp);
        if (!g_file_delete(descend, nullptr, &error)){
          gchar *tmp = g_file_get_path(descend); 
          g_warning("file %s cannot be deleted",
                    tmp);
          g_free(tmp);
        }
        On_scope_exit{release_g_error(error);};
      }
      g_object_unref(info);
      info = g_file_enumerator_next_file(enumerator, nullptr, &error);
      release_g_error(error);
    }
  }
}

std::string QuiddityManager_Impl::get_name() const {
  return name_;
}

void QuiddityManager_Impl::register_classes() {
  // registering quiddities
  abstract_factory_.register_class<AudioTestSource>
      (AudioTestSource::switcher_doc_.get_class_name(),
       &AudioTestSource::switcher_doc_);
  abstract_factory_.register_class<CreateRemoveSpy>
      (CreateRemoveSpy::switcher_doc_.get_class_name(),
       &CreateRemoveSpy::switcher_doc_);
  abstract_factory_.register_class<ExternalShmdataWriter>
      (ExternalShmdataWriter::switcher_doc_.get_class_name(),
       &ExternalShmdataWriter::switcher_doc_);
  abstract_factory_.register_class<GstVideoConverter>
      (GstVideoConverter::switcher_doc_.get_class_name(),
       &GstVideoConverter::switcher_doc_);
  abstract_factory_.register_class<GstVideoEncoder>
      (GstVideoEncoder::switcher_doc_.get_class_name(),
       &GstVideoEncoder::switcher_doc_);
  abstract_factory_.register_class<GstAudioEncoder>
      (GstAudioEncoder::switcher_doc_.get_class_name(),
       &GstAudioEncoder::switcher_doc_);
  // abstract_factory_.register_class<GstParseToBinSrc>
  //     (GstParseToBinSrc::switcher_doc_.get_class_name(),
  //      &GstParseToBinSrc::switcher_doc_);
  abstract_factory_.register_class<HTTPSDPDec>
      (HTTPSDPDec::switcher_doc_.get_class_name(),
       &HTTPSDPDec::switcher_doc_);
  abstract_factory_.register_class<Logger>
      (Logger::switcher_doc_.get_class_name(),
       &Logger::switcher_doc_);
  abstract_factory_.register_class<PropertyMapper>
      (PropertyMapper::switcher_doc_.get_class_name(),
       &PropertyMapper::switcher_doc_);
  abstract_factory_.register_class<RtpSession>
      (RtpSession::switcher_doc_.get_class_name(),
       &RtpSession::switcher_doc_);
  // abstract_factory_.register_class<ShmdataFromGDPFile>
  //     (ShmdataFromGDPFile::switcher_doc_.get_class_name(),
  //      &ShmdataFromGDPFile::switcher_doc_);
  // abstract_factory_.register_class<ShmdataToFile>
  //     (ShmdataToFile::switcher_doc_.get_class_name(),
  //      &ShmdataToFile::switcher_doc_);
  abstract_factory_.register_class<StringDictionary>
      (StringDictionary::switcher_doc_.get_class_name(),
       &StringDictionary::switcher_doc_);
  abstract_factory_.register_class<Uridecodebin>
      (Uridecodebin::switcher_doc_.get_class_name(),
       &Uridecodebin::switcher_doc_);
  abstract_factory_.register_class<VideoTestSource>
      (VideoTestSource::switcher_doc_.get_class_name(),
       &VideoTestSource::switcher_doc_);
}

std::vector<std::string> QuiddityManager_Impl::get_classes(){
  return abstract_factory_.get_keys();
}

void QuiddityManager_Impl::make_classes_doc() {
  std::vector<QuiddityDocumentation *> docs =
      abstract_factory_.get_classes_documentation();
  classes_doc_->reset();
  classes_doc_->begin_object();
  classes_doc_->set_member_name("classes");
  classes_doc_->begin_array();
  for (auto &it : docs)
    classes_doc_->add_node_value(it->get_json_root_node());
  classes_doc_->end_array();
  classes_doc_->end_object();
}

std::string QuiddityManager_Impl::get_classes_doc() {
  return classes_doc_->get_string(true);
}

std::string QuiddityManager_Impl::get_class_doc(const std::string &class_name) {
  if (abstract_factory_.key_exists(class_name))
    return JSONBuilder::get_string(
        abstract_factory_.get_class_documentation(class_name)->get_json_root_node(),
        true);

  return "{ \"error\":\"class not found\" }";
}

bool QuiddityManager_Impl::class_exists(const std::string &class_name) {
  return abstract_factory_.key_exists(class_name);
}

bool QuiddityManager_Impl::init_quiddity(Quiddity::ptr quiddity) {
  quiddity->set_manager_impl(me_.lock());
  if (!quiddity->init())
    return false;

  quiddities_[quiddity->get_name()] = quiddity;

  if (creation_hook_ != nullptr)
    (*creation_hook_) (quiddity->get_name(), creation_hook_user_data_);

  return true;
}

// for use of the "get description by class" methods
std::string
QuiddityManager_Impl::create_without_hook(const std::string &quiddity_class) {
  if (!class_exists(quiddity_class))
    return std::string();
  std::string name = quiddity_class + std::to_string(quiddity_created_counter_);
  quiddity_created_counter_++;
  Quiddity::ptr quiddity = abstract_factory_.create(quiddity_class, name);
  if (nullptr == quiddity.get())
    return "{\"error\":\"cannot make quiddity\"}";
  quiddity->set_manager_impl(me_.lock());
  quiddity->set_name(name);
  if (!quiddity->init())
    return "{\"error\":\"cannot init quiddity class\"}";
  quiddities_[name] = quiddity;
  return name;
}

std::string QuiddityManager_Impl::create(const std::string &quiddity_class) {
  if (!class_exists(quiddity_class))
    return std::string();
  std::string name = quiddity_class + std::to_string(quiddity_created_counter_);
  quiddity_created_counter_++;
  Quiddity::ptr quiddity = abstract_factory_.create(quiddity_class, name);
  if (quiddity.get() != nullptr) {
    quiddity->set_name(name);
    if (!init_quiddity(quiddity)) {
      g_debug("initialization of %s failled", quiddity_class.c_str());
      return std::string();
    }
  }
  return name;
}

std::string
QuiddityManager_Impl::create(const std::string &quiddity_class,
                             const std::string &raw_nick_name) {
  std::string nick_name = StringUtils::replace_chars(
      raw_nick_name,
      {';', '/', '[', ']', '&', '~', '*', '`', '#', '$', '|','\'', '"', '<', '>'},
      ' ');
  if (!class_exists(quiddity_class) || nick_name.empty())
    return std::string();
  auto it = quiddities_.find(nick_name);
  if (quiddities_.end() != it) {
    g_warning("cannot create a quiddity named %s, name is already taken",
              nick_name.c_str());
    return std::string();
  }
  Quiddity::ptr quiddity = abstract_factory_.create(quiddity_class,
                                                    nick_name);
  if (!quiddity) {
    g_warning("abstract factory failled to create %s (class %s)",
              nick_name.c_str(), quiddity_class.c_str());
    return std::string();
  }
  quiddity->set_name(nick_name);
  if (!init_quiddity(quiddity)) {
    g_debug("initialization of %s with name %s failled",
            quiddity_class.c_str(), quiddity->get_name().c_str());
    return std::string();
  }
  return nick_name;
}

std::vector<std::string> QuiddityManager_Impl::get_instances() const {
  std::vector<std::string> res;
  for (auto &it : quiddities_)
    res.push_back(it.first);
  return res;
}

bool QuiddityManager_Impl::has_instance(const std::string &name) const {
  return quiddities_.end() != quiddities_.find(name);
}

std::string QuiddityManager_Impl::get_quiddities_description() {
  JSONBuilder::ptr descr(new JSONBuilder());
  descr->reset();
  descr->begin_object();
  descr->set_member_name("quiddities");
  descr->begin_array();
  std::vector<std::string> quids = get_instances();
  for (std::vector<std::string>::iterator it = quids.begin();
       it != quids.end(); ++it) {
    descr->begin_object();
    std::shared_ptr<Quiddity> quid = get_quiddity(*it);
    descr->add_string_member("id", quid->get_name().c_str());
    descr->add_string_member("class",
                             quid->get_documentation()->get_class_name().
                             c_str());
    descr->end_object();
  }

  descr->end_array();
  descr->end_object();

  return descr->get_string(true);
}

std::string
QuiddityManager_Impl::get_quiddity_description(const std::string &nick_name) {
  auto it = quiddities_.find(nick_name);
  if (quiddities_.end() == it)
    return "{ \"error\":\"quiddity not found\"}";

  JSONBuilder::ptr descr(new JSONBuilder());
  descr->reset();
  descr->begin_object();
  descr->add_string_member("id", nick_name.c_str());
  // FIXME should use json node
  descr->add_string_member("class",
                           it->second->get_documentation()->get_class_name().c_str());
  descr->end_object();
  return descr->get_string(true);
}

Quiddity::ptr QuiddityManager_Impl::get_quiddity(const std::string &quiddity_name) {
  auto it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == it) {
    g_debug("quiddity %s not found, cannot provide ptr",
            quiddity_name.c_str());
    Quiddity::ptr empty_quiddity_ptr;
    return empty_quiddity_ptr;
  }
  return it->second;
}

// for use of "get description by class" methods only
//FIXME this should be the same method as remove
bool QuiddityManager_Impl::remove_without_hook(const std::string &quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_debug("(%s) quiddity %s not found for removing", name_.c_str(),
            quiddity_name.c_str());
    return false;
  }
  for (auto &it : property_subscribers_)
    it.second->unsubscribe(q_it->second);
  for (auto &it : signal_subscribers_)
    it.second->unsubscribe(q_it->second);
  quiddities_.erase(quiddity_name);
  return true;
}

bool QuiddityManager_Impl::remove(const std::string &quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("(%s) quiddity %s not found for removing", name_.c_str(),
              quiddity_name.c_str());
    return false;
  }
  for (auto &it : property_subscribers_)
    it.second->unsubscribe(q_it->second);
  for (auto &it : signal_subscribers_)
    it.second->unsubscribe(q_it->second);
  quiddities_.erase(quiddity_name);
  if (removal_hook_ != nullptr)
    (*removal_hook_) (quiddity_name.c_str(), removal_hook_user_data_);
  return true;
}

std::string
QuiddityManager_Impl::
get_properties_description(const std::string &quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get description of properties",
              quiddity_name.c_str());
    return std::string();
  }
  return q_it->second->get_properties_description();
}

std::string
QuiddityManager_Impl::get_property_description(const std::string &quiddity_name,
                                               const std::string &property_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get description of properties",
              quiddity_name.c_str());
    return std::string();
  }
  return q_it->second->get_property_description(property_name);
}

std::string
QuiddityManager_Impl::get_properties_description_by_class(const std::string &class_name) {
  if (!class_exists(class_name))
    return "{\"error\":\"class not found\"}";
  std::string quid_name = create_without_hook(class_name);
  if (quid_name.empty())
    return "{\"error\":\"cannot get property because the class cannot be instanciated\"}";
  std::string descr = get_properties_description(quid_name);
  remove_without_hook(quid_name);
  return descr;
}

std::string
QuiddityManager_Impl::get_property_description_by_class(const std::string &class_name,
                                                        const std::string &property_name) {
  if (!class_exists(class_name))
    return "{\"error\":\"class not found\"}";
  std::string quid_name = create_without_hook(class_name);
  if (quid_name.empty())
    return "{\"error\":\"cannot get property because the class cannot be instanciated\"}";
  std::string descr = get_property_description(quid_name, property_name);
  remove_without_hook(quid_name);
  return descr;
}

bool
QuiddityManager_Impl::set_property(const std::string &quiddity_name,
                                   const std::string &property_name,
                                   const std::string &property_value) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot set property",
              quiddity_name.c_str());
    return false;
  }
  return q_it->second->set_property(property_name.c_str(),
                                    property_value.c_str());
}

// higher level subscriber
bool
QuiddityManager_Impl::make_property_subscriber(const std::string &subscriber_name,
                                               QuiddityPropertySubscriber::Callback cb,
                                               void *user_data) {
  auto it = property_subscribers_.find(subscriber_name);
  if (property_subscribers_.end() != it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s already exists",
              subscriber_name.c_str());
    return false;
  }
  QuiddityPropertySubscriber::ptr subscriber(std::make_shared<QuiddityPropertySubscriber>());
  subscriber->set_manager_impl(me_.lock());
  subscriber->set_name(subscriber_name.c_str());
  subscriber->set_user_data(user_data);
  subscriber->set_callback(cb);
  property_subscribers_[subscriber_name] = subscriber;
  return true;
}

bool
QuiddityManager_Impl::
remove_property_subscriber(const std::string &subscriber_name) {
  auto it = property_subscribers_.find(subscriber_name);
  if (property_subscribers_.end() == it) {
    g_warning("a subscriber named %s does not exists\n",
              subscriber_name.c_str());
    return false;
  }
  property_subscribers_.erase(it);
  return true;
}

bool
QuiddityManager_Impl::subscribe_property(const std::string &subscriber_name,
                                         const std::string &quiddity_name,
                                         const std::string &property_name) {
  auto it = property_subscribers_.find(subscriber_name);
  if (property_subscribers_.end() == it) {
    g_warning
        ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
         subscriber_name.c_str());
    return false;
  }
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot subscribe to property",
              quiddity_name.c_str());
    return false;
  }
  return it->second->subscribe(q_it->second, property_name);
}

bool
QuiddityManager_Impl::unsubscribe_property(const std::string &subscriber_name,
                                           const std::string &quiddity_name,
                                           const std::string &property_name) {
  auto it = property_subscribers_.find(subscriber_name);
  if (property_subscribers_.end() == it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s does not exists\n",
              subscriber_name.c_str());
    return false;
  }
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot subscribe to property",
              quiddity_name.c_str());
    return false;
  }
  return it->second->unsubscribe(q_it->second, property_name);
}

std::vector<std::string>
QuiddityManager_Impl::list_property_subscribers() const {
  std::vector<std::string> res;
  for (auto &it : property_subscribers_)
    res.push_back(it.first);
  return res;
}

std::vector<std::pair<std::string, std::string>>
    QuiddityManager_Impl::list_subscribed_properties(const std::string &subscriber_name) {
  auto it = property_subscribers_.find(subscriber_name);
  if (property_subscribers_.end() == it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s does not exists\n",
              subscriber_name.c_str());
    return std::vector<std::pair<std::string, std::string>>();
  }
  return it->second->list_subscribed_properties();
}

std::string QuiddityManager_Impl::list_property_subscribers_json() {
  return "{\"error\":\"to be implemented\"}";  // FIXME (list_property_subscriber_json)
}

std::string
QuiddityManager_Impl::list_subscribed_properties_json(const std::string &subscriber_name) {
  auto subscribed_props = list_subscribed_properties(subscriber_name);

  JSONBuilder *doc = new JSONBuilder();
  doc->reset();
  doc->begin_object();
  doc->set_member_name("subscribed properties");
  doc->begin_array();
  for (auto &it : subscribed_props) {
    doc->begin_object();
    doc->add_string_member("quiddity", it.first.c_str());
    doc->add_string_member("property", it.second.c_str());
    doc->end_object();
  }
  doc->end_array();
  doc->end_object();

  return doc->get_string(true);
}

// lower level subscriber
bool
QuiddityManager_Impl::subscribe_property_glib(const std::string &quiddity_name,
                                              const std::string &property_name,
                                              Property::Callback cb,
                                              void *user_data) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot subscribe to property",
              quiddity_name.c_str());
    return false;
  }
  return q_it->second->subscribe_property(property_name.c_str(), cb, user_data);
}

bool
QuiddityManager_Impl::unsubscribe_property_glib(const std::string &quiddity_name,
                                                const std::string &property_name,
                                                Property::Callback cb,
                                                void *user_data) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot unsubscribe to property",
              quiddity_name.c_str());
    return false;
  }
  return q_it->second->unsubscribe_property(property_name.c_str(), cb, user_data);
}

std::string
QuiddityManager_Impl::get_property(const std::string &quiddity_name,
                                   const std::string &property_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get property",
              quiddity_name.c_str());
    return "{\"error\":\"quiddity not found\"}";
  }
  return q_it->second->get_property(property_name.c_str());
}

bool
QuiddityManager_Impl::has_property(const std::string &quiddity_name,
                                   const std::string &property_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_debug("quiddity %s not found", quiddity_name.c_str());
    return false;
  }
  return q_it->second->has_property(property_name);
}

bool
QuiddityManager_Impl::has_method(const std::string &quiddity_name,
                                 const std::string &method_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_debug("quiddity %s not found", quiddity_name.c_str());
    return false;
  }
  return q_it->second->has_method(method_name);
}

bool
QuiddityManager_Impl::invoke(const std::string &quiddity_name,
                             const std::string &method_name,
                             std::string **return_value,
                             const std::vector<std::string> &args) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_debug("quiddity %s not found, cannot invoke", quiddity_name.c_str());
    if (return_value != nullptr)
      *return_value = new std::string();
    return false;
  }
  if (!q_it->second->has_method(method_name)) {
    g_debug("method %s not found, cannot invoke", method_name.c_str());
    if (return_value != nullptr)
      *return_value = new std::string();
    return false;
  }
  return q_it->second->invoke_method(method_name, return_value, args);
}

std::string
QuiddityManager_Impl::get_methods_description(const std::string &quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get description of methods",
              quiddity_name.c_str());
    return "{\"error\":\"quiddity not found\"}";
  }
  return q_it->second->get_methods_description();
}

std::string
QuiddityManager_Impl::get_method_description(const std::string &quiddity_name,
                                             const std::string &method_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get description of methods",
              quiddity_name.c_str());
    return "{\"error\":\"quiddity not found\"}";
  }
  
  return q_it->second->get_method_description(method_name);
}

std::string QuiddityManager_Impl::get_methods_description_by_class(const std::string &class_name) {
  if (!class_exists(class_name))
    return "{\"error\":\"class not found\"}";
  const std::string &quid_name = create_without_hook(class_name);
  const std::string &descr = get_methods_description(quid_name);
  remove_without_hook(quid_name);
  return descr;
}

std::string
QuiddityManager_Impl::
get_method_description_by_class(const std::string &class_name,
                                const std::string &method_name) {
  if (!class_exists(class_name))
    return "{\"error\":\"class not found\"}";
  const std::string &quid_name = create_without_hook(class_name);
  const std::string &descr = get_method_description(quid_name, method_name);
  remove_without_hook(quid_name);
  return descr;
}

// *** signals
std::string QuiddityManager_Impl::get_signals_description(const std::string &quiddity_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get signals description",
              quiddity_name.c_str());
    return std::string();
  }
  return q_it->second->get_signals_description();
}

std::string
QuiddityManager_Impl::get_signal_description(const std::string &quiddity_name,
                                             const std::string &signal_name) {
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot get signal description",
              quiddity_name.c_str());
    return std::string();
  }
  return q_it->second->get_signal_description(signal_name);
}

std::string QuiddityManager_Impl::get_signals_description_by_class(const std::string &class_name) {
  if (!class_exists(class_name))
    return "{\"error\":\"class not found\"}";
  std::string quid_name = create_without_hook(class_name);
  if (quid_name.empty())
    return "{\"error\":\"cannot get signal description because the class cannot be instanciated\"}";
  std::string descr = get_signals_description(quid_name);
  remove_without_hook(quid_name);
  return descr;
}

std::string QuiddityManager_Impl::get_signal_description_by_class(const std::string &class_name,
                                                                  const std::string &signal_name) {
  if (!class_exists(class_name))
    return "{\"error\":\"class not found\"}";
  std::string quid_name = create_without_hook(class_name);
  if (quid_name.empty())
    return "{\"error\":\"cannot get signal because the class cannot be instanciated\"}";
  std::string descr = get_signal_description(quid_name, signal_name);
  remove_without_hook(quid_name);
  return descr;
}

// higher level subscriber
void QuiddityManager_Impl::mute_signal_subscribers(bool muted) {
  for (auto &it : signal_subscribers_)
    it.second->mute(muted);
}

void QuiddityManager_Impl::mute_property_subscribers(bool muted) {
  for (auto &it : property_subscribers_)
    it.second->mute(muted);
}

bool
QuiddityManager_Impl::make_signal_subscriber(const std::string &subscriber_name,
                                             QuidditySignalSubscriber::OnEmittedCallback cb,
                                             void *user_data) {
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

bool
QuiddityManager_Impl::
remove_signal_subscriber(const std::string &subscriber_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning("QuiddityManager_Impl, a subscriber named %s does not exists\n",
         subscriber_name.c_str());
    return false;
  }
  signal_subscribers_.erase(it);
  return true;
}

bool
QuiddityManager_Impl::subscribe_signal(const std::string &subscriber_name,
                                       const std::string &quiddity_name,
                                       const std::string &signal_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning
        ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
         subscriber_name.c_str());
    return false;
  }
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot subscribe to signal",
              quiddity_name.c_str());
    return false;
  }
  return it->second->subscribe(q_it->second, signal_name);
}

bool
QuiddityManager_Impl::unsubscribe_signal(const std::string &subscriber_name,
                                         const std::string &quiddity_name,
                                         const std::string &signal_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning
        ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
         subscriber_name.c_str());
    return false;
  }
  auto q_it = quiddities_.find(quiddity_name);
  if (quiddities_.end() == q_it) {
    g_warning("quiddity %s not found, cannot subscribe to signal",
              quiddity_name.c_str());
    return false;
  }
  return it->second->unsubscribe(q_it->second, signal_name);
}

std::vector<std::string> QuiddityManager_Impl::list_signal_subscribers() const {
  std::vector<std::string> res;
  for (auto &it : signal_subscribers_)
    res.push_back(it.first);
  return res;
}

std::vector<std::pair<std::string, std::string>>
    QuiddityManager_Impl::list_subscribed_signals(const std::string &subscriber_name) {
  auto it = signal_subscribers_.find(subscriber_name);
  if (signal_subscribers_.end() == it) {
    g_warning
        ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
         subscriber_name.c_str());
    std::vector<std::pair<std::string, std::string>>empty;
    return empty;
  }
  return it->second->list_subscribed_signals();
}

std::string QuiddityManager_Impl::list_signal_subscribers_json() {
  return "{\"error\":\"to be implemented\"}";  // FIXME (list_signal_subscriber_json)
}

std::string
QuiddityManager_Impl::list_subscribed_signals_json(const std::string &subscriber_name) {
  auto subscribed_sigs = list_subscribed_signals(subscriber_name);

  JSONBuilder *doc = new JSONBuilder();
  doc->reset();
  doc->begin_object();
  doc->set_member_name("subscribed signals");
  doc->begin_array();
  for (auto &it : subscribed_sigs) {
    doc->begin_object();
    doc->add_string_member("quiddity", it.first.c_str());
    doc->add_string_member("signal", it.second.c_str());
    doc->end_object();
  }
  doc->end_array();
  doc->end_object();

  return doc->get_string(true);
}

bool
QuiddityManager_Impl::set_created_hook(quiddity_created_hook hook,
                                       void *user_data) {
  if (creation_hook_ != nullptr)
    return false;
  creation_hook_ = hook;
  creation_hook_user_data_ = user_data;
  return true;
}

bool
QuiddityManager_Impl::set_removed_hook(quiddity_removed_hook hook,
                                       void *user_data) {
  if (removal_hook_ != nullptr)
    return false;
  removal_hook_ = hook;
  removal_hook_user_data_ = user_data;
  return true;
}

void QuiddityManager_Impl::reset_create_remove_hooks() {
  creation_hook_ = nullptr;
  removal_hook_ = nullptr;
  creation_hook_user_data_ = nullptr;
  removal_hook_user_data_ = nullptr;
}


GMainContext *QuiddityManager_Impl::get_g_main_context() {
  return mainloop_->get_main_context();
}

bool QuiddityManager_Impl::load_plugin(const char *filename) {
  PluginLoader::ptr plugin = std::make_shared<PluginLoader>();
  if (!plugin->load(filename))
    return false;
  std::string class_name = plugin->get_class_name();
  // close the old one if exists
  auto it = plugins_.find(class_name);
  if (plugins_.end() != it) {
    g_debug("closing old plugin for reloading (class: %s)",
            class_name.c_str());
    close_plugin(class_name);
  }
  abstract_factory_.register_class_with_custom_factory(
      class_name,
      plugin->get_doc(),
      plugin->create_,
      plugin->destroy_);
  plugins_[class_name] = plugin;
  return true;
}

void QuiddityManager_Impl::close_plugin(const std::string &class_name) {
  abstract_factory_.unregister_class(class_name);
  plugins_.erase(class_name);
}

bool
QuiddityManager_Impl::scan_directory_for_plugins(const char
                                                 *directory_path) {
  GFile *dir = g_file_new_for_commandline_arg(directory_path);
  gboolean res;
  GError *error;
  GFileEnumerator *enumerator;
  GFileInfo *info;
  error = nullptr;
  enumerator =
      g_file_enumerate_children(dir, "*",
                                G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS,
                                nullptr, &error);
  if (!enumerator)
    return false;
  error = nullptr;
  info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  while ((info) && (!error)) {
  GFile *descend = g_file_get_child(dir, g_file_info_get_name(info));
  char *absolute_path = g_file_get_path(descend);  // g_file_get_relative_path (dir, descend);
    // trying to load the module
    if (g_str_has_suffix(absolute_path, ".so")
        || g_str_has_suffix(absolute_path, ".dylib")) {
      g_debug("loading module %s", absolute_path);
      load_plugin(absolute_path);
    }
    g_free(absolute_path);
    g_object_unref(descend);
    info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  }
  error = nullptr;
  res = g_file_enumerator_close(enumerator, nullptr, &error);
  if (res != TRUE)
    g_debug("scanning dir: file enumerator not properly closed");
  if (error != nullptr)
    g_debug("scanning dir: error not nullptr");
  g_object_unref(dir);

  classes_doc_.reset(new JSONBuilder());
  make_classes_doc();

  return true;
}

std::string
QuiddityManager_Impl::get_info(const std::string &nick_name,
                               const std::string &path) {
  auto it = quiddities_.find(nick_name);
  if (quiddities_.end() == it)
    return "{ \"error\":\"quiddity not found\"}";
  return quiddities_[nick_name]->get_info(path);
}

}  // namespace switcher
