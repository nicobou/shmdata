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

#ifndef __SWITCHER_METHOD_CONTAINER_H__
#define __SWITCHER_METHOD_CONTAINER_H__

#include <map>
#include <memory>
#include <string>
#include "../../infotree/information-tree.hpp"
#include "../../logger/logged.hpp"
#include "../../utils/bool-any.hpp"
#include "../../utils/counter-map.hpp"
#include "../../utils/is-specialization-of.hpp"
#include "../../utils/method-trait.hpp"
#include "../../utils/type-name-registry.hpp"
#include "./method.hpp"

namespace switcher {
namespace quiddity {
namespace method {
using on_tree_grafted_cb_t = std::function<void(const std::string& key)>;
using on_tree_pruned_cb_t = std::function<void(const std::string& key)>;
using on_method_created_cb_t = std::function<void(const std::string& name)>;
using on_method_removed_cb_t = std::function<void(const std::string& name)>;
using on_method_enabled_cb_t = std::function<void(const std::string& name)>;
using on_method_disabled_cb_t = std::function<void(const std::string& name)>;

class MBag : public log::Logged {
  friend class Bundle;  // replacing some methods like replace and delete
 public:
  MBag() = delete;
  // ctor will own tree and write into .method.
  MBag(log::Base* log,
       InfoTree::ptr tree,
       on_tree_grafted_cb_t on_tree_grafted_cb,
       on_tree_pruned_cb_t on_tree_pruned_cb,
       on_method_created_cb_t on_method_created,
       on_method_removed_cb_t on_method_removed,
       on_method_enabled_cb_t on_method_enabled,
       on_method_disabled_cb_t on_method_disabled);

  // ------------- use
  // return 0 if id is not found
  meth_id_t get_id(const std::string& id) const;
  std::string get_name(meth_id_t id) const;
  std::vector<std::pair<std::string, method::meth_id_t>> get_ids() const;
  std::map<meth_id_t, std::string> get_names() const;

  // ----------- add/remove/update (you should prefer makers for adding)
  bool remove(meth_id_t meth_id);
  bool enable(meth_id_t meth_id);
  bool disable(meth_id_t meth_id, const std::string& why);

  // invoke
  template <typename M>
  decltype(auto) invoke(meth_id_t id, typename method_trait<M>::args_t&& t) const {
    const auto& meth_it = meths_.find(id);
    return static_cast<Method<M>*>(meth_it->second.get())->invoke_from_tuple(t);
  }

  BoolLog invoke_str(meth_id_t id, const std::string& serialized_tuple) const {
    return meths_.find(id)->second.get()->invoke(serialized_tuple);
  }

  BoolAny invoke_any(meth_id_t id, const std::string& serialized_tuple) const {
    return meths_.find(id)->second.get()->invoke_any(serialized_tuple);
  }

  // make
  template <typename M>
  BoolLog validate_method_documentation(InfoTree* tree) {
    if (!tree) return BoolLog(false, "null information tree");
    if (tree->empty()) return BoolLog(false, "information tree is empty");
    if (!tree->branch_has_data("name")) return BoolLog(false, "name is missing");
    if (!tree->branch_has_data("description")) return BoolLog(false, "description is missing");
    if (method_trait<M>::arity > 0 && !tree->branch_is_array("arguments"))
      return BoolLog(false, "arguments is not an array");
    for (int i = 0; i < method_trait<M>::arity; ++i) {
      if (!tree->branch_has_data("arguments." + std::to_string(i) + ".long name."))
        return BoolLog(false, "long name is missing for argument " + std::to_string(i));
      if (!tree->branch_has_data("arguments." + std::to_string(i) + ".description"))
        return BoolLog(false, "description is missing for argument " + std::to_string(i));
    }
    return BoolLog(true);
  }

  template <typename M>
  meth_id_t make_method(const std::string& strid, InfoTree::ptr tree, M&& meth) {
    auto valid = validate_method_documentation<M>(tree.get());
    if (!valid) {
      error("method % could not be installed: %", strid, valid.msg());
      return 0;
    }
    meths_.emplace(std::make_pair(++counter_, std::make_unique<Method<M>>(std::forward<M>(meth))));
    ids_.emplace(strid, counter_);
    auto key = std::string("method.") + strid;
    tree_->graft(key, tree);
    tree_->graft(key + ".id", InfoTree::make(strid));
    tree_->graft(key + ".meth_id", InfoTree::make(counter_));
    tree_->graft(key + ".parent", InfoTree::make(std::string()));
    tree_->graft(key + ".enabled", InfoTree::make(true));
    tree_->graft(key + ".order", InfoTree::make(counter_ * 20));

    auto ret_type = TypeNameRegistry::get_name<typename method_trait<M>::return_t>();
    if (ret_type.empty())
      tree_->graft(key + ".return type", InfoTree::make());
    else
      tree_->graft(key + ".return type", InfoTree::make(ret_type));

    auto args_types = TypeNameRegistry::get_names<typename method_trait<M>::args_t>();
    auto i = 0;
    for (auto& it : args_types) {
      tree_->graft(key + ".arguments." + std::to_string(i) + ".type", InfoTree::make(it));
      ++i;
    }

    if (on_tree_grafted_cb_) on_tree_grafted_cb_(key);
    on_method_created_(strid);
    return counter_;
  }

 private:
  meth_id_t counter_{0};
  std::map<meth_id_t, std::unique_ptr<MethodBase>> meths_{};
  std::map<std::string, meth_id_t> ids_{};
  std::map<meth_id_t, std::string> strids_{};
  InfoTree::ptr tree_;
  on_tree_grafted_cb_t on_tree_grafted_cb_;
  on_tree_pruned_cb_t on_tree_pruned_cb_;
  on_method_created_cb_t on_method_created_;
  on_method_removed_cb_t on_method_removed_;
  on_method_enabled_cb_t on_method_enabled_;
  on_method_disabled_cb_t on_method_disabled_;
  CounterMap suborders_{};
};

}  // namespace method
}  // namespace quiddity
}  // namespace switcher
#endif
