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

#include "../../utils/scope-exit.hpp"
#include "../container.hpp"
#include "../quid-id-t.hpp"
#include "switcher/infotree/json-serializer.hpp"

namespace switcher {
namespace quiddity {
namespace bundle {
Quiddity* create(quiddity::Config&& conf) {
  return new Bundle(std::forward<quiddity::Config>(conf));
}
void destroy(Quiddity* quiddity) { delete quiddity; }

Bundle::~Bundle() {
  quitting_ = true;
  for (const auto& reg : registered_sigs_) {
    reg.quid_->sig<MPtr(&signal::SBag::unsubscribe_by_name)>(reg.sig_name_, reg.reg_id_);
  }
}

Bundle::Bundle(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      conf_(conf),
      manager_(Switcher::make_switcher(conf_.nickname_)) {
  for (auto& it : qcontainer_->get_switcher()->qfactory_.get_plugin_dirs()) {
    manager_->qfactory_.scan_dir(it);
  }

  manager_->conf<MPtr(&Configuration::set)>(
      qcontainer_->get_switcher()->conf<MPtr(&Configuration::get)>());

  if (!config<MPtr(&InfoTree::branch_has_data)>("pipeline")) {
    LOGGER_WARN(this->logger, "bundle description is missing the pipeline description");
    is_valid_ = false;
    return;
  }
  pipeline_ = config<MPtr(&InfoTree::branch_get_value)>("pipeline").copy_as<std::string>();
  auto spec = bundle::DescriptionParser(pipeline_, std::vector<std::string>());
  if (!spec) {
    LOGGER_WARN(this->logger,
                "{} : error parsing the pipeline ({})",
                get_nickname(),
                spec.get_parsing_error());
    is_valid_ = false;
    return;
  }
  if (!make_quiddities(spec.get_quiddities())) {
    is_valid_ = false;
    return;
  }
  reader_quid_ = spec.get_reader_quid();

  // build the claw conf
  claw::OnConnect_t on_connect_cb;
  claw::OnDisconnect_t on_disconnect_cb;

  InfoTree::ptr con_tree = InfoTree::make();
  for (const auto& quid_nick : exposed_writer_quids_) {
    auto quid = manager_->qcontainer_->get_quiddity(manager_->qcontainer_->get_id(quid_nick));
    quid->connection_spec_tree_->cfor_each_in_array("writer", [&](const InfoTree* tree) {
      const auto label = tree->branch_get_value("label").as<std::string>();
      auto writer_tree = tree->get_copy();
      writer_tree->prune("swid");
      writer_tree->prune("from_swid");
      con_tree->graft("writer." + quid_nick, writer_tree);
      // change label in order to avoid collision among exposed writers
      con_tree->vgraft("writer." + quid_nick + ".label", quid_nick + "/" + label);
    });
  }
  if (!exposed_writer_quids_.empty()) {
    con_tree->tag_as_array("writer", true);
  }

  if (!reader_quid_.empty()) {
    auto quid = manager_->qcontainer_->get_quiddity(manager_->qcontainer_->get_id(reader_quid_));
    claw::sfid_t quid_sfid;
    quid->connection_spec_tree_->cfor_each_in_array("follower", [&](const InfoTree* tree) {
      const auto label = tree->branch_get_value("label").as<std::string>();
      auto follower_tree = tree->get_copy();
      quid_sfid = tree->branch_get_value("sfid").as<claw::sfid_t>();
      follower_tree->prune("sfid");
      follower_tree->prune("from_sfid");
      con_tree->graft("follower." + reader_quid_, follower_tree);
      // change label in order to avoid collision among exposed writers
      con_tree->vgraft("follower." + reader_quid_ + ".label", reader_quid_ + "/" + label);
    });
    con_tree->tag_as_array("follower", true);
    on_connect_cb = [this, quid, quid_sfid](const std::string& shmpath, claw::sfid_t) {
      return quid->claw_.on_connect_cb_(shmpath, quid_sfid);
    };
    on_disconnect_cb = [this, quid, quid_sfid](claw::sfid_t) {
      return quid->claw_.on_disconnect_cb_(quid_sfid);
    };
  }

  claw_ = claw::Claw(this,
                     claw::ConnectionSpec(infotree::json::serialize(con_tree.get())),
                     on_connect_cb,
                     on_disconnect_cb);
  // now force the shmdata writer path
  for (const auto& quid_nick : exposed_writer_quids_) {
    auto quid = manager_->qcontainer_->get_quiddity(manager_->qcontainer_->get_id(quid_nick));
    quid->connection_spec_tree_->cfor_each_in_array("writer", [&](const InfoTree* tree) {
      const auto label = tree->branch_get_value("label").as<std::string>();
      const auto quid_swid = tree->branch_get_value("swid").as<claw::swid_t>();
      const auto bundle_swid = claw_.get_swid(quid_nick + "/" + label);
      claw_.forced_writer_shmpaths_.emplace(bundle_swid, quid->claw_.get_writer_shmpath(quid_swid));
    });
  }
}

bool Bundle::make_quiddities(const std::vector<bundle::quiddity_spec_t>& quids) {
  // first: check is start must be installed
  for (auto& quid : quids) {
    if (quid.expose_start) start_quids_.emplace_back(quid.name);
    if (quid.expose_shmw) exposed_writer_quids_.emplace_back(quid.name);
  }
  if (!start_quids_.empty()) {
    init_startable(this);
  }
  // create quiddities and set properties
  for (auto& quid : quids) {
    auto res = manager_->qcontainer_->create(quid.kind, quid.name, nullptr);
    if (!res) {
      LOGGER_WARN(
          this->logger, "internal manager failed to instantiate a quiddity of kind {}", quid.kind);
      return false;
    }
    std::string name = res.msg();

    for (auto& param : quid.params) {
      if (param.first == "started") {
        LOGGER_WARN(this->logger, "ignoring started property when building bundle");
        continue;
      }
      if (!param.second.empty() &&
          !res.get()->prop<MPtr(&property::PBag::set_str_str)>(param.first, param.second)) {
        LOGGER_WARN(this->logger,
                    "fail to set property {} to {} for quiddity {}",
                    param.first,
                    param.second,
                    name);
        return false;
      }
    }  // quiddity is created

    // registering quiddity properties
    auto quid_ptr = manager_->qcontainer_->get_quiddity(manager_->qcontainer_->get_id(name));
    on_tree_datas_.emplace_back(std::make_unique<on_tree_data_t>(this, name, quid_ptr.get(), quid));
    registered_sigs_.push_back(
        QuidRegSig(quid_ptr,
                   "on-tree-grafted",
                   quid_ptr->sig<MPtr(&signal::SBag::subscribe_by_name)>(
                       std::string("on-tree-grafted"),
                       [&, tree_data = on_tree_datas_.back().get()](InfoTree::ptr tree) {
                         Bundle::on_tree_grafted(tree->get_value().as<std::string>(), tree_data);
                       })));
    registered_sigs_.push_back(
        QuidRegSig(quid_ptr,
                   "on-tree-pruned",
                   quid_ptr->sig<MPtr(&signal::SBag::subscribe_by_name)>(
                       std::string("on-tree-pruned"),
                       [&, tree_data = on_tree_datas_.back().get()](InfoTree::ptr tree) {
                         Bundle::on_tree_pruned(tree->get_value().as<std::string>(), tree_data);
                       })));
    // mirroring property
    if (!quid.top_level && (quid.expose_prop || !quid.whitelisted_params.empty())) {
      auto group_name = name;
      if (!quid.group_name.empty()) group_name = quid.group_name;
      pmanage<MPtr(&property::PBag::make_group)>(
          group_name, group_name, std::string("Properties for ") + name);
    }
    // We need to sort the list so that groups are created first or we could lose some properties.
    auto props = quid_ptr->props_.get_ids();
    std::partition(props.begin(),
                   props.end(),
                   [&quid_ptr](const std::pair<std::string, property::prop_id_t>& prop) {
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

        pmanage<MPtr(&property::PBag::mirror_property_from)>(
            name + "/" + prop.first, parent_strid, &quid_ptr->props_, prop.second);
      }
    }

    quiddity_removal_cb_ids_.push_back(
        manager_->qcontainer_->register_removal_cb([this](quiddity::qid_t id) {
          LOGGER_DEBUG(
              this->logger,
              "The bundle {} was destroyed because one of its quiddities ({}) was destroyed",
              nickname_,
              manager_->qcontainer_->get_nickname(id));
          // We only self destruct it once so we unregister all the removal callbacks.
          manager_->qcontainer_->reset_create_remove_cb();
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
        auto& qcontainer = context->self_->manager_->qcontainer_;
        for (auto& it : context->quid_spec_.connects_to_) {
          auto quid = qcontainer->get_quiddity(qcontainer->get_id(it));
          // WARNING test against the first follower sfid, bundle need to be extended
          // specification of which follower to connect in order to allow internal
          // connection to dedicated sfid
          auto sfid = quid->claw<MPtr(&quiddity::claw::Claw::get_sfids)>();
          if (sfid.empty()) {
            LOGGER_WARN(quid->logger,
                        "BUG in bundle, no follower available when connecting from quiddity {}",
                        quid->get_nickname());
            continue;
          }
          if (!quid->claw<MPtr(&quiddity::claw::Claw::sfid_can_do_shmtype)>(sfid[0], caps)) {
            LOGGER_WARN(context->self_->logger,
                        "ERROR: bundle specification error: {} cannot connect with {} (caps is {})",
                        context->quid_spec_.name,
                        it,
                        caps);
            continue;
          }

          quid->claw<MPtr(&quiddity::claw::Claw::connect_raw)>(sfid[0], shm_match[1]);
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

        context->self_->pmanage<MPtr(&property::PBag::mirror_property_from)>(
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
      if (!context->self_->pmanage<MPtr(&property::PBag::remove)>(
              context->self_->pmanage<MPtr(&property::PBag::get_id)>(context->quid_name_ + "/" +
                                                                     prop_name))) {
        LOGGER_WARN(context->self_->logger,
                    "BUG removing property ({}) deleted from a quiddity ({}) in a bundle ({})",
                    prop_name,
                    context->quid_name_,
                    context->self_->get_nickname());
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
      auto qcontainer = context->self_->manager_->qcontainer_;
      for (auto& it : to_disconnect) {
        auto quid = qcontainer->get_quiddity(qcontainer->get_id(it.first));
        if (!quid) continue;
        // WARNING disconnecting the first sfid only because specification of which sfid to connect
        // is missing
        const auto sfids = quid->claw<MPtr(&quiddity::claw::Claw::get_sfids)>();
        if (sfids.empty()) {
          LOGGER_WARN(quid->logger,
                      "BUG in bundle, no follower available when disconnecting from quiddity {}",
                      quid->get_nickname());
        }
        quid->claw<MPtr(&quiddity::claw::Claw::disconnect)>(sfids[0]);
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
    if (!manager_->qcontainer_->get_quiddity(manager_->qcontainer_->get_id(it))
             ->prop<MPtr(&property::PBag::set_str_str)>("started", "true")) {
      LOGGER_WARN(this->logger, "fail to set start {}", it);
      return false;
    }
  }
  return true;
}

bool Bundle::stop() {
  for (auto& it : start_quids_) {
    if (!manager_->qcontainer_->get_quiddity(manager_->qcontainer_->get_id(it))
             ->prop<MPtr(&property::PBag::set_str_str)>("started", "false")) {
      LOGGER_WARN(this->logger, "fail to set start {}", it);
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

}  // namespace bundle
}  // namespace quiddity
}  // namespace switcher
