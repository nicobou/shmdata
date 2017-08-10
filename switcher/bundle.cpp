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

#include "./bundle.hpp"
#include <regex>
#include "./logger-forwarder.hpp"
#include "./scope-exit.hpp"

namespace switcher {

namespace bundle {
Quiddity* create(QuiddityConfiguration&& conf) {
  return new Bundle(std::forward<QuiddityConfiguration>(conf));
}
void destroy(Quiddity* quiddity) { delete quiddity; }
}  // namespace bundle

Bundle::~Bundle() { quitting_ = true; }

Bundle::Bundle(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      conf_(conf),
      shmcntr_(static_cast<Quiddity*>(this)),
      manager_(Switcher::make_switcher<LoggerForwarder>(conf_.name_, conf_.log_)) {
  for (auto& it : qcontainer_->get_plugin_dirs()) {
    manager_->scan_directory_for_plugins(it);
  }

  manager_->qcontainer_->configurations_ = qcontainer_->configurations_;

  if (!config<MPtr(&InfoTree::branch_has_data)>("pipeline")) {
    warning("bundle description is missing the pipeline description");
    is_valid_ = false;
    return;
  }
  pipeline_ = config<MPtr(&InfoTree::branch_get_value)>("pipeline").copy_as<std::string>();
  auto spec = bundle::DescriptionParser(pipeline_, std::vector<std::string>());
  if (!spec) {
    warning("% : error parsing the pipeline (s)", get_name(), spec.get_parsing_error());
    is_valid_ = false;
    return;
  }
  if (!make_quiddities(spec.get_quiddities())) {
    is_valid_ = false;
    return;
  }
  reader_quid_ = spec.get_reader_quid();
  if (!reader_quid_.empty()) {
    shmcntr_.install_connect_method(
        [this](const std::string& shmpath) {
          std::string* return_value = nullptr;
          manager_->qcontainer_->get_quiddity(reader_quid_)
              ->invoke_method("connect", &return_value, {shmpath});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        [this](const std::string& shmpath) {
          std::string* return_value = nullptr;
          manager_->qcontainer_->get_quiddity(reader_quid_)
              ->invoke_method("disconnect", &return_value, {shmpath});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        [this]() {
          std::string* return_value = nullptr;
          manager_->qcontainer_->get_quiddity(reader_quid_)
              ->invoke_method("disconnect-all", &return_value, {});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        [this](const std::string& caps) {
          std::string* return_value = nullptr;
          manager_->qcontainer_->get_quiddity(reader_quid_)
              ->invoke_method("can-sink-caps", &return_value, {caps});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        manager_->qcontainer_->get_quiddity(reader_quid_)
            ->tree<MPtr(&InfoTree::branch_get_value)>("shmdata.max_reader")
            .copy_as<unsigned int>());
  }
}

bool Bundle::make_quiddities(const std::vector<bundle::quiddity_spec_t>& quids) {
  // first: check is start must be installed
  for (auto& quid : quids) {
    if (quid.expose_start) start_quids_.emplace_back(quid.name);
  }
  if (!start_quids_.empty()) {
    init_startable(this);
  }
  // create quiddities and set properties
  for (auto& quid : quids) {
    std::string name;

    name = manager_->create(quid.type, quid.name);

    if (name.empty()) {
      warning("internal manager failed to instantiate a quiddity of type %", quid.type);
      return false;
    }
    for (auto& param : quid.params) {
      if (param.first == "started") {
        warning("ignoring started property when building bundle");
        continue;
      }
      if (!param.second.empty() &&
          !manager_->use_prop<MPtr(&PContainer::set_str_str)>(name, param.first, param.second)) {
        warning("fail to set property % to % for quiddity %", param.first, param.second, name);
        return false;
      }
    }  // quiddity is created

    // registering quiddity properties
    auto quid_ptr = manager_->qcontainer_->get_quiddity(name);
    on_tree_datas_.emplace_back(std::make_unique<on_tree_data_t>(this, name, quid_ptr.get(), quid));
    quid_ptr->sig<MPtr(&SContainer::subscribe_by_name)>(
        std::string("on-tree-grafted"),
        [&, tree_data = on_tree_datas_.back().get() ](InfoTree::ptr tree) {
          Bundle::on_tree_grafted(tree->get_value().as<std::string>(), tree_data);
        });
    quid_ptr->sig<MPtr(&SContainer::subscribe_by_name)>(
        std::string("on-tree-pruned"),
        [&, tree_data = on_tree_datas_.back().get() ](InfoTree::ptr tree) {
          Bundle::on_tree_pruned(tree->get_value().as<std::string>(), tree_data);
        });
    // mirroring property
    if (!quid.top_level && (quid.expose_prop || !quid.whitelisted_params.empty())) {
      auto group_name = name;
      if (!quid.group_name.empty()) group_name = quid.group_name;
      pmanage<MPtr(&PContainer::make_group)>(
          group_name, group_name, std::string("Properties for ") + name);
    }
    // We need to sort the list so that groups are created first or we could lose some properties.
    auto props = quid_ptr->props_.get_ids();
    std::partition(props.begin(),
                   props.end(),
                   [&quid_ptr](const std::pair<std::string, PContainer::prop_id_t>& prop) {
                     auto type = quid_ptr->tree<MPtr(&InfoTree::branch_get_value)>(
                         "property." + prop.first + ".type");
                     return type.is_null() || type.copy_as<std::string>() == "group";
                   });
    for (auto& prop : props) {
      if (property_is_displayed(quid, prop.first)) {
        std::string parent_strid;
        auto parent =
            quid_ptr->tree<MPtr(&InfoTree::branch_get_value)>("property." + prop.first + ".parent");
        if (!quid.top_level || (parent.not_null() && !parent.copy_as<std::string>().empty())) {
          parent_strid = quid.group_name.empty() ? name : quid.group_name;
        }

        pmanage<MPtr(&PContainer::mirror_property_from)>(
            name + "/" + prop.first, parent_strid, &quid_ptr->props_, prop.second);
      }
    }

    quiddity_removal_cb_ids_.push_back(
        manager_->register_removal_cb([this](const std::string& quid_name) {
          debug("The bundle % was destroyed because one of its quiddities (%) was destroyed",
                name_,
                quid_name);
          // We only self destruct it once so we unregister all the removal callbacks.
          manager_->reset_create_remove_cb();
          self_destruct();
        }));
  }  // finished dealing with this quid specification
  return true;
}

void Bundle::on_tree_grafted(const std::string& key, void* user_data) {
  auto context = static_cast<on_tree_data_t*>(user_data);
  static std::regex connections_wr_rgx("\\.?shmdata.writer\\.([^.]+)");
  if (!context->quid_spec_.connects_to_.empty()) {
    std::smatch shm_match;
    std::regex_search(key, shm_match, connections_wr_rgx);
    if (!shm_match.empty()) {
      // shm_match[0] contains the matched sequence,
      // shm_match[1] contains the sub sequence localized between brackets
      std::string caps = context->quid_->information_tree_->branch_get_value(
          static_cast<std::string>(shm_match[0]) + ".caps");
      if (!caps.empty()) {
        for (auto& it : context->quid_spec_.connects_to_) {
          if (!context->self_->manager_->qcontainer_->get_quiddity(it)->invoke_method(
                  "can-sink-caps", nullptr, {caps})) {
            context->self_->message(
                "ERROR: bundle specification error: % cannot connect with % (caps is %)",
                context->quid_spec_.name,
                it,
                caps);
            context->self_->warning(
                "ERROR: bundle specification error: % cannot connect with % (caps is %)",
                context->quid_spec_.name,
                it,
                caps);
            continue;
          }
          context->self_->manager_->qcontainer_->get_quiddity(it)->invoke_method(
              "connect", nullptr, {shm_match[1]});
          {
            std::lock_guard<std::mutex> lock(context->self_->connected_shms_mtx_);
            context->self_->connected_shms_.push_back(std::make_pair(it, shm_match[1]));
          }
        }
        std::swap(context->quid_spec_.connects_to_, context->quid_spec_.connected_to_);
        // WARNING do not return here. If you do so the signal will not be forwarded, resulting in
        // shmdata not being exposed
      }
    }
  }

  static std::regex exposed_wr_rgx("\\.?shmdata\\.writer\\..*");
  if (context->quid_spec_.expose_shmw) {
    if (std::regex_match(key, exposed_wr_rgx)) {
      context->self_->graft_tree(key, context->quid_->information_tree_->get_tree(key), true);
      return;
    }
  }

  static std::regex exposed_rd_rgx("\\.?shmdata\\.reader\\..*");
  if (context->quid_spec_.expose_shmr) {
    if (std::regex_match(key, exposed_rd_rgx)) {
      context->self_->graft_tree(key, context->quid_->information_tree_->get_tree(key), true);
      return;
    }
  }

  static std::regex prop_rgx("\\.?property\\.([^.]*).*");
  std::smatch prop_match;
  if (std::regex_search(key, prop_match, prop_rgx)) {
    std::string prop_name = prop_match[1];
    if (!property_is_displayed(context->quid_spec_, prop_name)) {
      return;
    }
    static std::regex prop_created_rgx("\\.?property\\.[^.]*");
    if (std::regex_match(key, prop_created_rgx)) {
      if (property_is_displayed(context->quid_spec_, prop_name)) {
        std::string parent_strid;
        auto parent = context->quid_->tree<MPtr(&InfoTree::branch_get_value)>(
            "property." + prop_name + ".parent");
        if (!context->quid_spec_.top_level ||
            (parent.not_null() && !parent.copy_as<std::string>().empty())) {
          parent_strid = context->quid_name_;
        }

        context->self_->pmanage<MPtr(&PContainer::mirror_property_from)>(
            context->quid_name_ + "/" + prop_name,
            parent_strid,
            &context->quid_->props_,
            context->quid_->props_.get_id(prop_name));
      }
    }
    static std::regex prop_rename("\\.?property.");
    context->self_->graft_tree(
        std::regex_replace(key, prop_rename, std::string("$&") + context->quid_name_ + "/"),
        context->quid_->information_tree_->get_tree(key),
        true);
    return;
  }

  // We catch the events we don't want to forward.
  if (!context->quid_spec_.expose_shmr && std::regex_match(key, exposed_rd_rgx)) return;
  if (!context->quid_spec_.expose_shmw && std::regex_match(key, exposed_wr_rgx)) return;
  if (context->quid_spec_.connects_to_.empty() && std::regex_match(key, connections_wr_rgx)) return;

  // And then if there is a custom branch in the tree we forward the graft event.
  context->self_->graft_tree(key, context->quid_->information_tree_->get_tree(key), true);
}

void Bundle::on_tree_pruned(const std::string& key, void* user_data) {
  auto context = static_cast<on_tree_data_t*>(user_data);

  static std::regex exposed_rd_rgx("\\.?shmdata\\.reader\\..*");
  if (context->quid_spec_.expose_shmr) {
    if (std::regex_match(key, exposed_rd_rgx)) {
      context->self_->prune_tree(key, true);
      return;
    }
  }

  static std::regex prop_rgx("\\.?property\\.([^.]*).*");
  std::smatch prop_match;
  if (std::regex_search(key, prop_match, prop_rgx)) {
    std::string prop_name = prop_match[1];
    if (!property_is_displayed(context->quid_spec_, prop_name)) {
      return;
    }
    static std::regex prop_deleted_rgx("\\.?property\\.[^.]*");
    if (std::regex_match(key, prop_deleted_rgx)) {
      if (!context->self_->pmanage<MPtr(&PContainer::remove)>(
              context->self_->pmanage<MPtr(&PContainer::get_id)>(context->quid_name_ + "/" +
                                                                 prop_name))) {
        context->self_->warning(
            "BUG removing property (%) deleted from a quiddity (%) in a bundle (%)",
            prop_name,
            context->quid_name_,
            context->self_->get_name());
      }
    }
    static std::regex prop_rename("\\.?property\\.");
    context->self_->prune_tree(
        std::regex_replace(key, prop_rename, std::string("$&") + context->quid_name_ + "/"), true);
    return;
  }

  static std::regex wr_rgx("\\.?shmdata\\.writer\\.([^.]+)");
  std::smatch shm_match;
  std::regex_search(key, shm_match, wr_rgx);
  if (!shm_match.empty()) {
    std::vector<std::pair<std::string, std::string>> to_disconnect;
    {
      std::lock_guard<std::mutex> lock(context->self_->connected_shms_mtx_);
      auto rm = std::remove_if(
          context->self_->connected_shms_.begin(),
          context->self_->connected_shms_.end(),
          [&](const std::pair<std::string /*quid_name*/, std::string /*shmpath*/>& connection) {
            if (connection.second == shm_match[1]) {
              to_disconnect.push_back(std::make_pair(connection.first, connection.second));
              return true;
            }
            return false;
          });
      if (rm != context->self_->connected_shms_.end()) {
        context->self_->connected_shms_.erase(rm, context->self_->connected_shms_.end());
      }
    }  // release connected_shms_mtx_
    if (context->quid_spec_.expose_shmw) {
      context->self_->prune_tree(key, true);
    }
    std::swap(context->quid_spec_.connects_to_, context->quid_spec_.connected_to_);
    if (!context->self_->quitting_.load()) {
      for (auto& it : to_disconnect) {
        auto quid = context->self_->manager_->qcontainer_->get_quiddity(it.first);
        if (!quid) continue;
        quid->invoke_method("disconnect", nullptr, {it.second});
      }
    }
    return;
  }

  // We catch the events we don't want to forward.
  if (!context->quid_spec_.expose_shmr && std::regex_match(key, exposed_rd_rgx)) return;

  // And then if there is a custom branch in the tree we forward the prune event.
  context->self_->prune_tree(key, true);
}

bool Bundle::start() {
  for (auto& it : start_quids_) {
    if (!manager_->qcontainer_->get_quiddity(it)->prop<MPtr(&PContainer::set_str_str)>("started",
                                                                                       "true")) {
      warning("fail to set start %", it);
      return false;
    }
  }
  return true;
}

bool Bundle::stop() {
  for (auto& it : start_quids_) {
    if (!manager_->qcontainer_->get_quiddity(it)->prop<MPtr(&PContainer::set_str_str)>("started",
                                                                                       "false")) {
      warning("fail to set start %", it);
      return false;
    }
  }
  return true;
}

bool Bundle::property_is_displayed(bundle::quiddity_spec_t quid_spec, std::string property_name) {
  return (!(quid_spec.expose_start && property_name == "started") &&
          ((quid_spec.expose_prop &&
            quid_spec.blacklisted_params.end() == std::find(quid_spec.blacklisted_params.begin(),
                                                            quid_spec.blacklisted_params.end(),
                                                            property_name)) ||
           (!quid_spec.expose_prop &&
            quid_spec.whitelisted_params.end() != std::find(quid_spec.whitelisted_params.begin(),
                                                            quid_spec.whitelisted_params.end(),
                                                            property_name))));
}

}  // namespace switcher
