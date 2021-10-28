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

#include <string.h>
#include <filesystem>
#include <fstream>
#include "./switcher.hpp"
#include "./session/session.hpp"
#include "./gst/utils.hpp"
#include "./infotree/json-serializer.hpp"
#include "./quiddity/bundle/bundle.hpp"
#include "./quiddity/bundle/description-parser.hpp"
#include "./utils/file-utils.hpp"
#include "./utils/scope-exit.hpp"

namespace switcher {

namespace fs = std::filesystem;

std::string Switcher::get_name() const { return name_; }

std::string Switcher::get_switcher_version() const { return SWITCHER_VERSION_STRING; }
void Switcher::set_control_port(const int port) { control_port_ = port; }

int Switcher::get_control_port() const { return control_port_; }

std::string Switcher::get_switcher_caps() const {
  auto caps_str = "switcher-name=(string)" + get_name();
  if (get_control_port() > 0) {
    caps_str += ",control-port=(int)" + std::to_string(get_control_port());
  }
  
  return caps_str;
}

void Switcher::reset_state(bool remove_created_quiddities) {
  if (remove_created_quiddities) {
    for (auto& quid : qcontainer_->get_ids()) {
      if (quiddities_at_reset_.cend() ==
          std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid)) {
        qcontainer_->remove(quid);
      }
    }
  }
  quiddities_at_reset_ = qcontainer_->get_ids();
}

bool Switcher::load_state(InfoTree* state) {
  if (!state) return false;
  // trees
  auto quiddities_user_data = state->get_tree("userdata.");
  auto quiddities = state->get_tree(".quiddities");
  auto properties = state->get_tree(".properties");
  auto readers = state->get_tree(".readers");
  auto custom_states = state->get_tree(".custom_states");
  auto nicknames = state->get_tree(".nicknames");

  // making quiddities
  if (quiddities) {
    auto quids = quiddities->get_child_keys(".");
    // creating quiddities
    for (auto& it : quids) {
      std::string quid_kind = quiddities->branch_get_value(it);
      auto created = qcontainer_->create(quid_kind, it, nullptr);
      if (!created) {
        log_->message("ERROR:error creating quiddity % (kind %): ", it, quid_kind, created.msg());
        log_->warning("error creating quiddity % (kind %): ", it, quid_kind, created.msg());
      }
    }

    // loading custom state
    for (auto& it : quids) {
      if (0 == qcontainer_->get_id(it)) continue;
      if (custom_states && !custom_states->empty()) {
        qcontainer_->get_quiddity(qcontainer_->get_id(it))->on_loading(custom_states->get_tree(it));
      } else {
        qcontainer_->get_quiddity(qcontainer_->get_id(it))->on_loading(InfoTree::make());
      }
    }
  }

  // nicknames
  if (nicknames) {
    for (auto& it : nicknames->get_child_keys(".")) {
      std::string nickname = nicknames->branch_get_value(it);
      auto nicknamed_quid = qcontainer_->get_quiddity(qcontainer_->get_id(it));
      if (!nicknamed_quid || !nicknamed_quid->set_nickname(nickname)) {
        log_->message("ERROR:error applying nickname % for %", nickname, it);
        log_->warning("error applying nickname % for %", nickname, it);
      }
    }
  }

  // applying properties
  std::vector<std::string> quid_to_start;
  if (properties) {
    auto quids = properties->get_child_keys(".");
    for (auto& name : quids) {
      auto props = properties->get_child_keys(name);
      for (auto& prop : props) {
        if (prop == "started" && properties->branch_get_value(name + ".started")) {
          quid_to_start.push_back(name);
        } else {
          auto quid = qcontainer_->get_quiddity(qcontainer_->get_id(name));
          if (!quid || !quid->prop<MPtr(&quiddity::property::PBag::set_str_str)>(
                           prop, Any::to_string(properties->branch_get_value(name + "." + prop)))) {
            log_->message("ERROR:failed to apply value, quiddity is %, property is %, value is %",
                          name,
                          prop,
                          Any::to_string(properties->branch_get_value(name + "." + prop)));
            log_->warning("failed to apply value, quiddity is %, property is %, value is %",
                          name,
                          prop,
                          Any::to_string(properties->branch_get_value(name + "." + prop)));
          }
        }
      }
    }
  }

  // applying user data to quiddities
  if (quiddities_user_data) {
    auto quids = quiddities_user_data->get_child_keys(".");
    for (auto& it : quids) {
      auto quid_id = qcontainer_->get_id(it);
      if (0 != quid_id) {
        auto child_keys = quiddities_user_data->get_child_keys(it);
        for (auto& kit : child_keys) {
          qcontainer_->user_data<MPtr(&InfoTree::graft)>(
              quid_id, kit, quiddities_user_data->get_tree(it + "." + kit));
        }
      }
    }
  }
  // starting quiddities
  for (auto& name : quid_to_start) {
    auto quid = qcontainer_->get_quiddity(qcontainer_->get_id(name));
    if (!quid) {
      log_->message("ERROR:failed to get quiddity %", name);
      log_->warning("failed to get quiddity %", name);
      continue;
    }
    if (!quid || !quid->prop<MPtr(&quiddity::property::PBag::set_str_str)>("started", "true")) {
      log_->message("ERROR:failed to start quiddity %", name);
      log_->warning("failed to start quiddity %", name);
    }
  }

  // Connect to raw shmdata
  if (readers) {
    auto quids = readers->get_child_keys(".");
    for (auto& quid : quids) {
      auto quid_id = qcontainer_->get_id(quid);
      auto quidreaders = readers->get_child_keys(quid + ".shm_from_quid");
      auto connect_quid_id =
          qcontainer_->meths<MPtr(&quiddity::method::MBag::get_id)>(quid_id, "connect-quid");
      for (auto& reader : quidreaders) {
        qcontainer_->meths<MPtr(
            &quiddity::method::MBag::invoke<std::function<bool(std::string, std::string)>>)>(
            quid_id,
            connect_quid_id,
            std::make_tuple(Any::to_string(readers->branch_get_value(quid + ".shm_from_quid." +
                                                                     reader + ".name")),
                            Any::to_string(readers->branch_get_value(quid + ".shm_from_quid." +
                                                                     reader + ".suffix"))));
      }
      auto rawreaders = readers->get_child_keys(quid + ".raw_shm");
      for (auto& reader : rawreaders) {
        qcontainer_->meths<MPtr(&quiddity::method::MBag::invoke<std::function<bool(std::string)>>)>(
            qcontainer_->get_id(quid),
            qcontainer_->meths<MPtr(&quiddity::method::MBag::get_id)>(qcontainer_->get_id(quid),
                                                                      "connect"),
            std::make_tuple(Any::to_string(readers->branch_get_value(quid + "." + reader))));
      }
    }
  }

  // on_loaded
  if (quiddities) {
    auto quids = quiddities->get_child_keys(".");
    for (auto& it : quids) {
      auto quid_id = qcontainer_->get_id(it);
      if (0 == quid_id) continue;
      qcontainer_->get_quiddity(quid_id)->on_loaded();
    }
  }
  return true;
}

InfoTree::ptr Switcher::get_state() const {
  auto quiddities = qcontainer_->get_ids();
  InfoTree::ptr tree = InfoTree::make();

  // saving per-quiddity information
  for (auto& quid_id : quiddities) {
    auto nick = qcontainer_->get_nickname(quid_id);
    // saving custom tree if some is provided
    auto custom_tree = qcontainer_->get_quiddity(quid_id)->on_saving();
    if (custom_tree && !custom_tree->empty())
      tree->graft(".custom_states." + nick, std::move(custom_tree));

    // name and kind
    if (quiddities_at_reset_.cend() ==
        std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid_id)) {
      tree->graft(".quiddities." + nick,
                  InfoTree::make(qcontainer_->get_quiddity(quid_id)->get_kind()));
    }

    // nicknames
    if (quiddities_at_reset_.cend() ==
        std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid_id)) {
      tree->graft(".nicknames." + nick, InfoTree::make(nick));
    }

    // user data
    auto quid = qcontainer_->get_quiddity(quid_id);
    auto quid_user_data_tree = quid->user_data<MPtr(&InfoTree::get_tree)>(".");
    if (!quid_user_data_tree->empty()) {
      tree->graft(".userdata." + nick, quid_user_data_tree);
    }

    // writable property values
    quid->prop<MPtr(&quiddity::property::PBag::update_values_in_tree)>();
    auto props = quid->tree<MPtr(&InfoTree::get_child_keys)>("property");
    for (auto& prop : props) {
      // Don't save unwritable properties.
      if (!quid->tree<MPtr(&InfoTree::branch_get_value)>(std::string("property.") + prop +
                                                         ".writable"))
        continue;
      // Don't save properties with saving disabled.
      if (!quid->prop_is_saved(prop)) continue;
      tree->graft(".properties." + nick + "." + prop,
                  InfoTree::make(quid->tree<MPtr(&InfoTree::branch_get_value)>(
                      std::string("property.") + prop + ".value")));
    }

    // Record shmdata connections.
    // Ignore them if no connect-to methods is installed for this quiddity.
    if (0 == quid->meth<MPtr(&quiddity::method::MBag::get_id)>("connect-quid")) continue;

    auto readers = quid->tree<MPtr(&InfoTree::get_child_keys)>("shmdata.reader");
    int nb = 0;
    for (auto& reader : readers) {
      auto writer_info =
          quid->tree<MPtr(&InfoTree::branch_get_copy)>("shmdata.reader." + reader + ".writer");
      if (writer_info) {
        tree->graft(".readers." + nick + ".shm_from_quid.reader_" + std::to_string(++nb),
                    writer_info);
      } else {
        tree->graft(".readers." + nick + ".raw_shm.reader_" + std::to_string(++nb),
                    InfoTree::make(reader));
      }
    }
    tree->tag_as_array(".readers." + nick + ".shm_from_quid", true);
    tree->tag_as_array(".readers." + nick + ".raw_shm", true);
  }
  // calling on_saved callback
  for (auto& quid_id : quiddities) {
    qcontainer_->get_quiddity(quid_id)->on_saved();
  }

  return tree;
}

void Switcher::apply_gst_configuration() {
  auto configuration = conf_.get();
  for (const auto& plugin : configuration->get_child_keys("gstreamer.primary_priority")) {
    int priority = configuration->branch_get_value("gstreamer.primary_priority." + plugin);
    if (!gst::Initialized::set_plugin_as_primary(plugin, priority)) {
      log_->warning("Unable to find Gstreamer plugin '%'. Check if plugin is installed.", plugin);
    }
  }
}

bool Switcher::load_bundle_from_config(const std::string& bundle_description) {
  log_->debug("Receiving new bundles configuration: %", bundle_description);
  auto bundles = infotree::json::deserialize(bundle_description);
  if (!bundles) {
    log_->error("Invalid bundle configuration.");
    return false;
  }

  auto new_configuration = InfoTree::copy(conf_.get().get());
  bool bundles_added = false;
  for (const auto& bundle_name : bundles->get_child_keys("bundle")) {
    if (new_configuration.get()->branch_get_copy(".bundle." + bundle_name) !=
        InfoTree::make_null()) {
      log_->warning("Bundle '%' already exists. Skipping.", bundle_name);
      continue;
    }
    bundles_added = true;
    new_configuration->graft(".bundle." + bundle_name + ".",
                             bundles->branch_get_copy("bundle." + bundle_name));
  }

  if (bundles_added) {
    conf_.set(new_configuration);
    register_bundle_from_configuration();
    log_->debug("New bundles added");
  }
  return true;
}

void Switcher::register_bundle_from_configuration() {
  // registering bundle(s) as creatable kind
  auto quid_kinds = qfactory_.get_kinds();
  auto configuration = conf_.get();
  for (auto& it : configuration->get_child_keys("bundle")) {
    if (std::string::npos != it.find('_')) {
      log_->warning("underscores are not allowed for quiddity kinds (bundle name %)", it);
      continue;
    }
    std::string long_name = configuration->branch_get_value("bundle." + it + ".doc.long_name");
    std::string description = configuration->branch_get_value("bundle." + it + ".doc.description");
    std::string pipeline = configuration->branch_get_value("bundle." + it + ".pipeline");
    std::string is_missing;
    if (long_name.empty()) is_missing = "long_name";
    if (description.empty()) is_missing = "description";
    if (pipeline.empty()) is_missing = "pipeline";
    if (!is_missing.empty()) {
      log_->warning("% : % field is missing, cannot create new quiddity kind", it, is_missing);
      continue;
    }
    // check if the pipeline description is correct
    auto spec = quiddity::bundle::DescriptionParser(pipeline, quid_kinds);
    if (!spec) {
      log_->warning("% : error parsing the pipeline (%)", it, spec.get_parsing_error());
      continue;
    }
    // ok, bundle can be added
    // Bundle names must be unique
    if (!quiddity::DocumentationRegistry::get()->register_doc(
            it, quiddity::Doc(long_name, it, description, "n/a", "n/a"))) {
      log_->warning("bundle '%' already exists. Skipping.", it);
      continue;
    }

    qfactory_.register_kind_with_custom_factory(
        it, &quiddity::bundle::create, &quiddity::bundle::destroy);
    // making the new bundle kind available for next bundle definition:
    quid_kinds.push_back(it);
  }
}

void Switcher::remove_shm_zombies() const {
  auto files = fileutils::get_shmfiles_from_directory(this->get_shm_dir(),
                                                      this->get_shm_prefix() + this->get_name());
  for (auto& it : files) {
    auto res = fileutils::remove(it);
    if (!res) log_->warning("fail removing shmdata % (%)", it, res.msg());
  }
}

}  // namespace switcher
