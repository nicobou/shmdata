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
#include <future>
#include <regex>
#include "./scope-exit.hpp"

namespace switcher {

namespace bundle {
Quiddity* create(const std::string& name) { return new Bundle(name); }
void destroy(Quiddity* quiddity) { delete quiddity; }
}  // namespace bundle

Bundle::Bundle(const std::string& name)
    : shmcntr_(static_cast<Quiddity*>(this)), manager_(QuiddityManager::make_manager(name)) {}

bool Bundle::init() {
  if (!config<MPtr(&InfoTree::branch_has_data)>("pipeline")) {
    g_warning("bundle description is missing the pipeline description");
    return false;
  }
  pipeline_ = config<MPtr(&InfoTree::branch_get_value)>("pipeline").copy_as<std::string>();
  auto spec = bundle::DescriptionParser(pipeline_, std::vector<std::string>());
  if (!spec) {
    g_warning("%s : error parsing the pipeline (%s)",
              doc_getter_()->get_class_name().c_str(),
              spec.get_parsing_error().c_str());
    return false;
  }
  if (!make_quiddities(spec.get_quiddities())) {
    return false;
  }
  reader_quid_ = spec.get_reader_quid();
  if (!reader_quid_.empty()) {
    shmcntr_.install_connect_method(
        [this](const std::string& shmpath) {
          std::string* return_value = nullptr;
          manager_->invoke(reader_quid_, "connect", &return_value, {shmpath});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        [this](const std::string& shmpath) {
          std::string* return_value = nullptr;
          manager_->invoke(reader_quid_, "disconnect", &return_value, {shmpath});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        [this]() {
          std::string* return_value = nullptr;
          manager_->invoke(reader_quid_, "disconnect-all", &return_value, {});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        [this](const std::string& caps) {
          std::string* return_value = nullptr;
          manager_->invoke(reader_quid_, "can-sink-caps", &return_value, {caps});
          On_scope_exit {
            if (return_value) delete return_value;
          };
          return *return_value == "true";
        },
        manager_->use_tree<MPtr(&InfoTree::branch_get_value)>(reader_quid_, "shmdata.max_reader")
            .copy_as<unsigned int>());
  }
  return true;
}

QuiddityDocumentation* Bundle::get_documentation() {
  if (doc_getter_) return doc_getter_();
  return nullptr;
}

void Bundle::set_doc_getter(doc_getter_t doc_getter) { doc_getter_ = doc_getter; }

bool Bundle::make_quiddities(const std::vector<bundle::quiddity_spec_t>& quids) {
  for (auto& quid : quids) {
    std::string name;
    if (quid.name.empty()) {
      name = manager_->create(quid.type);
    } else {
      name = manager_->create(quid.type, quid.name);
    }
    if (name.empty()) {
      g_warning("internal manager failed to instantiate a quiddity of type %s", quid.type.c_str());
      return false;
    }
    for (auto& param : quid.params) {
      if (param.first == "started") {
        g_warning("ignoring started property when building bundle");
        continue;
      }
      if (!manager_->use_prop<MPtr(&PContainer::set_str_str)>(name, param.first, param.second)) {
        g_warning("fail to set property %s to %s for quiddity %s",
                  name.c_str(),
                  param.first.c_str(),
                  param.second.c_str());
        return false;
      }
    }  // quiddity is created

    // registering quiddity properties
    pmanage<MPtr(&PContainer::make_group)>(name, name, std::string("Properties for ") + name);
    auto quid_ptr = manager_->manager_impl_->get_quiddity(name);
    on_tree_datas_.emplace_back(std::make_unique<on_tree_data_t>(this, name, quid_ptr.get(), quid));
    quid_ptr->subscribe_signal(
        "on-tree-grafted", &Bundle::on_tree_grafted, on_tree_datas_.back().get());
    quid_ptr->subscribe_signal(
        "on-tree-pruned", &Bundle::on_tree_pruned, on_tree_datas_.back().get());
    for (auto& prop : quid_ptr->props_.get_ids()) {
      pmanage<MPtr(&PContainer::mirror_property_from)>(
          name + "/" + prop.first, name, &quid_ptr->props_, prop.second);
    }
  }  // finished dealing with this quid specification
  return true;
}

void Bundle::on_tree_grafted(const std::vector<std::string>& params, void* user_data) {
  auto context = static_cast<on_tree_data_t*>(user_data);

  if (!context->quid_spec_.connects_to_.empty()) {
    static std::regex wr_rgx("\\.?shmdata.writer\\.([^.]+)");
    std::smatch shm_match;
    std::regex_search(params[0], shm_match, wr_rgx);
    if (!shm_match.empty()) {
      // shm_match[0] contains the matched sequence,
      // shm_match[1] contains the sub sequence localized between brackets
      std::string caps = context->quid_->information_tree_->branch_get_value(
          static_cast<std::string>(shm_match[0]) + ".caps");
      if (!caps.empty()) {
        for (auto& it : context->quid_spec_.connects_to_) {
          if (!context->self_->manager_->invoke(it, "can-sink-caps", nullptr, {caps})) {
            g_message("ERROR: bundle specification error: %s cannot connect with %s (caps is %s)",
                      context->quid_spec_.name.c_str(),
                      it.c_str(),
                      caps.c_str());
            g_warning("ERROR: bundle specification error: %s cannot connect with %s (caps is %s)",
                      context->quid_spec_.name.c_str(),
                      it.c_str(),
                      caps.c_str());
            continue;
          }
          context->self_->manager_->invoke(it, "connect", nullptr, {shm_match[1]});
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

  if (context->quid_spec_.expose_shmw) {
    static std::regex wr_rgx("\\.?shmdata\\.writer\\..*");
    if (std::regex_match(params[0], wr_rgx)) {
      context->self_->graft_tree(
          params[0], context->quid_->information_tree_->get_tree(params[0]), true);
      return;
    }
  }

  if (context->quid_spec_.expose_shmr) {
    static std::regex rd_rgx("\\.?shmdata\\.reader\\..*");
    if (std::regex_match(params[0], rd_rgx)) {
      context->self_->graft_tree(
          params[0], context->quid_->information_tree_->get_tree(params[0]), true);
      return;
    }
  }

  static std::regex prop_rgx("\\.?property\\.([^.]*).*");
  std::smatch prop_match;
  if (std::regex_search(params[0], prop_match, prop_rgx)) {
    std::string prop_name = prop_match[1];
    static std::regex prop_created_rgx("\\.?property\\.[^.]*");
    if (std::regex_match(params[0], prop_created_rgx)) {
      context->self_->pmanage<MPtr(&PContainer::mirror_property_from)>(
          context->quid_name_ + "/" + prop_name,
          context->quid_name_,
          &context->quid_->props_,
          context->quid_->props_.get_id(prop_name));
    }
    static std::regex prop_rename("\\.?property.");
    context->self_->graft_tree(
        std::regex_replace(params[0], prop_rename, std::string("$&") + context->quid_name_ + "/"),
        context->quid_->information_tree_->get_tree(params[0]),
        true);
    return;
  }
}

void Bundle::on_tree_pruned(const std::vector<std::string>& params, void* user_data) {
  auto context = static_cast<on_tree_data_t*>(user_data);

  if (context->quid_spec_.expose_shmw) {
    static std::regex wr_rgx("\\.?shmdata\\.writer\\.([^.]+)");
    std::smatch shm_match;
    std::regex_search(params[0], shm_match, wr_rgx);
    if (!shm_match.empty()) {
      auto rm = std::remove_if(
          context->self_->connected_shms_.begin(),
          context->self_->connected_shms_.end(),
          [&](const std::pair<std::string /*quid_name*/, std::string /*shmpath*/>& connection) {
            if (connection.second == shm_match[1]) return true;
            return false;
          });
      for (auto& it = rm; it != context->self_->connected_shms_.end(); ++it) {
        // would be nice to have an invoke_async in manager_ :(
        context->self_->loop_.run_async([&, quid_name = it->first, shmpath = it->second ]() {
          context->self_->manager_->invoke(quid_name, "disconnect", nullptr, {shmpath});
        },
                                        nullptr);
      }
      context->self_->connected_shms_.erase(rm, context->self_->connected_shms_.end());
      context->self_->prune_tree(params[0], true);
      std::swap(context->quid_spec_.connects_to_, context->quid_spec_.connected_to_);
      return;
    }
  }

  if (context->quid_spec_.expose_shmr) {
    static std::regex rd_rgx("\\.?shmdata\\.reader\\..*");
    if (std::regex_match(params[0], rd_rgx)) {
      context->self_->prune_tree(params[0], true);
      return;
    }
  }

  static std::regex prop_rgx("\\.?property\\.([^.]*).*");
  std::smatch prop_match;
  if (std::regex_search(params[0], prop_match, prop_rgx)) {
    std::string prop_name = prop_match[1];
    static std::regex prop_deleted_rgx("\\.?property\\.[^.]*");
    if (std::regex_match(params[0], prop_deleted_rgx)) {
      if (!context->self_->pmanage<MPtr(&PContainer::remove)>(
              context->self_->pmanage<MPtr(&PContainer::get_id)>(context->quid_name_ + "/" +
                                                                 prop_name))) {
        g_warning("BUG removing property (%s) deleted from a quiddity (%s) in a bundle (%s)",
                  prop_name.c_str(),
                  context->quid_name_.c_str(),
                  context->self_->get_name().c_str());
      }
    }
    static std::regex prop_rename("\\.?property\\.");
    context->self_->prune_tree(
        std::regex_replace(params[0], prop_rename, std::string("$&") + context->quid_name_ + "/"),
        true);
    return;
  }
}

}  // namespace switcher
