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

#include "./switcher.hpp"
#include <string.h>
#include <fstream>
#include "./bundle-description-parser.hpp"
#include "./bundle.hpp"
#include "./file-utils.hpp"
#include "./gst-utils.hpp"
#include "./information-tree-json.hpp"
#include "./scope-exit.hpp"

namespace switcher {


std::string Switcher::get_name() const { return name_; }

std::string Switcher::get_switcher_version() const { return SWITCHER_VERSION_STRING; }

void Switcher::reset_state(bool remove_created_quiddities) {
  if (remove_created_quiddities) {
    for (auto& quid : qcontainer_->get_names()) {
      if (quiddities_at_reset_.cend() ==
          std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid)) {
        qcontainer_->remove(qcontainer_->get_id(quid));
      }
    }
  }
  quiddities_at_reset_ = qcontainer_->get_names();
}

bool Switcher::load_state(InfoTree::ptr state) {
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
      std::string quid_class = quiddities->branch_get_value(it);
      auto created = qcontainer_->create(quid_class, it);
      if (!created) {
        log_->message(
            "ERROR:error creating quiddity % (of type %): ", it, quid_class, created.msg());
        log_->warning("error creating quiddity % (of type %): ", it, quid_class, created.msg());
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
          if (!quid || !quid->prop<MPtr(&PContainer::set_str_str)>(
                           prop, Any::to_string(properties->branch_get_value(name + "." + prop))))
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
    if (!quid || !quid->prop<MPtr(&PContainer::set_str_str)>("started", "true")) {
      log_->message("ERROR:failed to start quiddity %", name);
      log_->warning("failed to start quiddity %", name);
    }
  }

  // Connect shmdata
  if (readers) {
    auto quids = readers->get_child_keys(".");
    for (auto& quid : quids) {
      auto tmpreaders = readers->get_child_keys(quid);
      for (auto& reader : tmpreaders) {
        qcontainer_->meths<MPtr(&MContainer::invoke<std::function<bool(std::string)>>)>(
            qcontainer_->get_id(quid),
            qcontainer_->meths<MPtr(&MContainer::get_id)>(qcontainer_->get_id(quid), "connect"),
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
  auto quiddities = qcontainer_->get_names();
  InfoTree::ptr tree = InfoTree::make();

  // saving per-quiddity information
  for (auto& quid_name : quiddities) {
    // saving custom tree if some is provided
    auto custom_tree = qcontainer_->get_quiddity(qcontainer_->get_id(quid_name))->on_saving();
    if (custom_tree && !custom_tree->empty())
      tree->graft(".custom_states." + quid_name, std::move(custom_tree));

    // name and class
    if (quiddities_at_reset_.cend() ==
        std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid_name)) {
      tree->graft(
          ".quiddities." + quid_name,
          InfoTree::make(qcontainer_->get_quiddity(qcontainer_->get_id(quid_name))->get_type()));
    }

    // nicknames
    if (quiddities_at_reset_.cend() ==
        std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid_name)) {
      tree->graft(".nicknames." + quid_name,
                  InfoTree::make(
                      qcontainer_->get_quiddity(qcontainer_->get_id(quid_name))->get_nickname()));
    }

    // user data
    auto quid = qcontainer_->get_quiddity(qcontainer_->get_id(quid_name));
    auto quid_user_data_tree = quid->user_data<MPtr(&InfoTree::get_tree)>(".");
    if (!quid_user_data_tree->empty()) {
      tree->graft(".userdata." + quid_name, quid_user_data_tree);
    }

    // writable property values
    quid->prop<MPtr(&PContainer::update_values_in_tree)>();
    auto props = quid->tree<MPtr(&InfoTree::get_child_keys)>("property");
    for (auto& prop : props) {
      // Don't save unwritable properties.
      if (!quid->tree<MPtr(&InfoTree::branch_get_value)>(std::string("property.") + prop +
                                                         ".writable"))
        continue;
      // Don't save properties with saving disabled.
      if (!quid->prop_is_saved(prop)) continue;
      tree->graft(".properties." + quid_name + "." + prop,
                  InfoTree::make(quid->tree<MPtr(&InfoTree::branch_get_value)>(
                      std::string("property.") + prop + ".value")));
    }

    // Record shmdata connections.
    // Ignore them if no connect-to methods is installed for this quiddity.
    if (0 == quid->meth<MPtr(&MContainer::get_id)>("connect")) continue;

    auto readers = quid->tree<MPtr(&InfoTree::get_child_keys)>("shmdata.reader");
    int nb = 0;
    for (auto& reader : readers) {
      if (!reader.empty()) {
        tree->graft(".readers." + quid_name + ".reader_" + std::to_string(++nb),
                    InfoTree::make(reader));
      }
    }
    tree->tag_as_array(".readers." + quid_name, true);
  }
  // calling on_saved callback
  for (auto& quid_name : quiddities) {
    qcontainer_->get_quiddity(qcontainer_->get_id(quid_name))->on_saved();
  }

  return tree;
}

void Switcher::register_bundle_from_configuration() {
  // registering bundle(s) as creatable class
  auto quid_types = qfactory_.get_class_list();
  auto configuration = conf_.get();
  for (auto& it : configuration->get_child_keys("bundle")) {
    if (std::string::npos != it.find('_')) {
      log_->warning("underscores are not allowed for quiddity types (bundle name %)", it);
      continue;
    }
    std::string long_name = configuration->branch_get_value("bundle." + it + ".doc.long_name");
    std::string category = configuration->branch_get_value("bundle." + it + ".doc.category");
    std::string tags = configuration->branch_get_value("bundle." + it + ".doc.tags");
    std::string description = configuration->branch_get_value("bundle." + it + ".doc.description");
    std::string pipeline = configuration->branch_get_value("bundle." + it + ".pipeline");
    std::string is_missing;
    if (long_name.empty()) is_missing = "long_name";
    if (category.empty()) is_missing = "category";
    if (tags.empty()) is_missing = "tags";
    if (description.empty()) is_missing = "description";
    if (pipeline.empty()) is_missing = "pipeline";
    if (!is_missing.empty()) {
      log_->warning("% : % field is missing, cannot create new quiddity type", it, is_missing);
      continue;
    }
    // check if the pipeline description is correct
    auto spec = bundle::DescriptionParser(pipeline, quid_types);
    if (!spec) {
      log_->warning("% : error parsing the pipeline (%)", it, spec.get_parsing_error());
      continue;
    }
    // ok, bundle can be added
    DocumentationRegistry::get()->register_doc(
        it, quid::Doc(long_name, it, category, tags, description, "n/a", "n/a"));

    qfactory_.register_class_with_custom_factory(it, &bundle::create, &bundle::destroy);
    // making the new bundle type available for next bundle definition:
    quid_types.push_back(it);
  }
}

void Switcher::remove_shm_zombies() const {
  auto files = FileUtils::get_shmfiles_from_directory(get_shm_dir(), get_shm_prefix());
  for (auto& it : files) {
    auto res = FileUtils::remove(it);
    if (!res) log_->warning("fail removing shmdata % (%)", it, res.msg());
  }
}

}  // namespace switcher
