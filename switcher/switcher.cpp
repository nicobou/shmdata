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
#include <fstream>
#include "./gst-utils.hpp"
#include "./information-tree-json.hpp"
#include "./scope-exit.hpp"
#include "./switcher.hpp"

namespace switcher {


std::string Switcher::get_name() const { return name_; }

void Switcher::reset_state(bool remove_created_quiddities) {
  if (remove_created_quiddities) {
    for (auto& quid : qcontainer_->get_instances()) {
      if (quiddities_at_reset_.cend() ==
          std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid)) {
        qcontainer_->remove(quid);
      }
    }
  }
  quiddities_at_reset_ = qcontainer_->get_instances();
}

void Switcher::init_gst() {
  if (!gst_is_initialized()) gst_init(nullptr, nullptr);
  GstRegistry* registry = gst_registry_get();
  // TODO add option for scanning a path
  gst_registry_scan_path(registry, "/usr/local/lib/gstreamer-1.0/");
  gst_registry_scan_path(registry, "/usr/lib/gstreamer-1.0/");
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
      auto created = create(quid_class, it);
      if (!created) {
        log_->warning("error creating quiddity % (of type %): ", it, quid_class, created.msg());
      }
    }

    // loading custom state
    for (auto& it : quids) {
      if (!qcontainer_->has_instance(it)) continue;
      if (custom_states && !custom_states->empty()) {
        qcontainer_->get_quiddity(it)->on_loading(custom_states->get_tree(it));
      } else {
        qcontainer_->get_quiddity(it)->on_loading(InfoTree::make());
      }
    }
  }

  // nicknames
  if (nicknames) {
    for (auto& it : nicknames->get_child_keys(".")) {
      std::string nickname = nicknames->branch_get_value(it);
      if (!set_nickname(it, nickname))
        log_->warning("error applying nickname % for %", nickname, it);
    }
  }

  // applying properties
  std::vector<std::string> quid_to_start;
  if (properties) {
    auto quids = properties->get_child_keys(".");
    for (auto& quid : quids) {
      auto props = properties->get_child_keys(quid);
      for (auto& prop : props) {
        if (prop == "started" && properties->branch_get_value(quid + ".started")) {
          quid_to_start.push_back(quid);
        } else {
          if (!use_prop<MPtr(&PContainer::set_str_str)>(
                  quid, prop, Any::to_string(properties->branch_get_value(quid + "." + prop))))
            log_->warning("failed to apply value, quiddity is %, property is %, value is %",
                          quid,
                          prop,
                          Any::to_string(properties->branch_get_value(quid + "." + prop)));
        }
      }
    }
  }

  // applying user data to quiddities
  if (quiddities_user_data) {
    auto quids = quiddities_user_data->get_child_keys(".");
    for (auto& it : quids) {
      if (qcontainer_->has_instance(it)) {
        auto child_keys = quiddities_user_data->get_child_keys(it);
        for (auto& kit : child_keys) {
          qcontainer_->user_data<MPtr(&InfoTree::graft)>(
              it, kit, quiddities_user_data->get_tree(it + "." + kit));
        }
      }
    }
  }
  // starting quiddities
  for (auto& quid : quid_to_start) {
    if (!use_prop<MPtr(&PContainer::set_str_str)>(quid, "started", "true")) {
      log_->warning("failed to start quiddity %", quid);
    }
  }

  // Connect shmdata
  if (readers) {
    auto quids = readers->get_child_keys(".");
    for (auto& quid : quids) {
      auto tmpreaders = readers->get_child_keys(quid);
      for (auto& reader : tmpreaders) {
        qcontainer_->meths<MPtr(&MContainer::invoke<std::function<bool(std::string)>>)>(
            quid,
            qcontainer_->meths<MPtr(&MContainer::get_id)>(quid, "connect"),
            std::make_tuple(Any::to_string(readers->branch_get_value(quid + "." + reader))));
      }
    }
  }

  // on_loaded
  if (quiddities) {
    auto quids = quiddities->get_child_keys(".");
    for (auto& it : quids) {
      if (!qcontainer_->has_instance(it)) continue;
      qcontainer_->get_quiddity(it)->on_loaded();
    }
  }
  return true;
}

InfoTree::ptr Switcher::get_state() const {
  auto quiddities = qcontainer_->get_instances();
  InfoTree::ptr tree = InfoTree::make();

  // saving per-quiddity information
  for (auto& quid_name : quiddities) {
    // saving custom tree if some is provided
    auto custom_tree = qcontainer_->get_quiddity(quid_name)->on_saving();
    if (custom_tree && !custom_tree->empty())
      tree->graft(".custom_states." + quid_name, std::move(custom_tree));

    // name and class
    if (quiddities_at_reset_.cend() ==
        std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid_name)) {
      tree->graft(".quiddities." + quid_name,
                  InfoTree::make(qcontainer_->get_quiddity(quid_name)->get_type()));
    }

    // nicknames
    if (quiddities_at_reset_.cend() ==
        std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid_name)) {
      tree->graft(".nicknames." + quid_name, InfoTree::make(get_nickname(quid_name)));
    }

    // user data
    auto quid_user_data_tree = user_data<MPtr(&InfoTree::get_tree)>(quid_name, ".");
    if (!quid_user_data_tree->empty()) {
      tree->graft(".userdata." + quid_name, user_data<MPtr(&InfoTree::get_tree)>(quid_name, "."));
    }
    // writable property values
    use_prop<MPtr(&PContainer::update_values_in_tree)>(quid_name);
    auto props = use_tree<MPtr(&InfoTree::get_child_keys)>(quid_name, "property");
    for (auto& prop : props) {
      // Don't save unwritable properties.
      if (!use_tree<MPtr(&InfoTree::branch_get_value)>(
              quid_name, std::string("property.") + prop + ".writable"))
        continue;
      // Don't save properties with saving disabled.
      if (!qcontainer_->get_quiddity(quid_name)->prop_is_saved(prop)) continue;
      tree->graft(".properties." + quid_name + "." + prop,
                  InfoTree::make(use_tree<MPtr(&InfoTree::branch_get_value)>(
                      quid_name, std::string("property.") + prop + ".value")));
    }

    // Record shmdata connections.
    // Ignore them if no connect-to methods is installed for this quiddity.
    if (0 == qcontainer_->get_quiddity(quid_name)->meth<MPtr(&MContainer::get_id)>("connect"))
      continue;

    auto readers = use_tree<MPtr(&InfoTree::get_child_keys)>(quid_name, "shmdata.reader");
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
    qcontainer_->get_quiddity(quid_name)->on_saved();
  }

  return tree;
}

void Switcher::auto_init(const std::string& quiddity_name) {
  Quiddity::ptr quidd = qcontainer_->get_quiddity(quiddity_name);
  if (!quidd) return;
  SwitcherWrapper::ptr wrapper = std::dynamic_pointer_cast<SwitcherWrapper>(quidd);
  if (wrapper) wrapper->set_quiddity_manager(me_.lock());
}

bool Switcher::scan_directory_for_plugins(const std::string& directory) {
  return qcontainer_->scan_directory_for_plugins(directory);
}

bool Switcher::load_configuration_file(const std::string& file_path) {
  return qcontainer_->load_configuration_file(file_path);
}

BoolLog Switcher::create(const std::string& quiddity_class, const std::string& nickname) {
  BoolLog res;
  invocation_loop_.run([&]() {
    res = qcontainer_->create(quiddity_class, nickname);
    if (res) auto_init(res.msg());
  });  // invocation_loop_.run
  return res;
}

BoolLog Switcher::remove(const std::string& quiddity_name) {
  BoolLog res;
  invocation_loop_.run(
      [&]() { res = qcontainer_->remove(quiddity_name); });  // invocation_loop_.run
  return res;
}

bool Switcher::has_quiddity(const std::string& name) { return qcontainer_->has_instance(name); }

std::vector<std::string> Switcher::get_classes() {
  std::vector<std::string> res;
  invocation_loop_.run([&]() { res = qcontainer_->get_classes(); });  // invocation_loop_.run
  return res;
}

InfoTree::ptr Switcher::get_classes_doc() {
  InfoTree::ptr res;
  invocation_loop_.run([&]() { res = qcontainer_->get_classes_doc(); });  // invocation_loop_.run
  return res;
}

InfoTree::ptr Switcher::get_quiddities_description() {
  InfoTree::ptr res;
  invocation_loop_.run(
      [&]() { res = qcontainer_->get_quiddities_description(); });  // invocation_loop_.run
  return res;
}

InfoTree::ptr Switcher::get_quiddity_description(const std::string& quiddity_name) {
  InfoTree::ptr res;
  invocation_loop_.run([&]() {
    res = qcontainer_->get_quiddity_description(quiddity_name);
  });  // invocation_loop_.run
  return res;
}

std::vector<std::string> Switcher::get_quiddities() const {
  std::vector<std::string> res;
  invocation_loop_.run([&]() { res = qcontainer_->get_instances(); });  // invocation_loop_.run
  return res;
}

std::string Switcher::get_nickname(const std::string& name) const {
  auto quid = qcontainer_->get_quiddity(name);
  if (!quid) {
    log_->debug("quiddity % not found", name);
    return std::string();
  }
  return quid->get_nickname();
}

bool Switcher::set_nickname(const std::string& name, const std::string& nickname) {
  auto quid = qcontainer_->get_quiddity(name);
  if (!quid) {
    log_->debug("quiddity % not found", name);
    return false;
  }
  return quid->set_nickname(nickname);
}

}  // namespace switcher
