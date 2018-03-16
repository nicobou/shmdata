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

#include <assert.h>
#include <map>
#include <memory>
#include <string>
#include "./counter-map.hpp"
#include "./information-tree.hpp"
#include "./is-specialization-of.hpp"
#include "./method-trait.hpp"
#include "./method2.hpp"
#include "./type-name-registry.hpp"

namespace switcher {
class MContainer {
  friend class Bundle;  // replacing some methods like replace and delete
 public:
  using meth_id_t = MethodBase::meth_id_t;
  using on_tree_grafted_cb_t = std::function<void(const std::string& key)>;
  using on_tree_pruned_cb_t = std::function<void(const std::string& key)>;
  MContainer() = delete;
  // ctor will own tree and write into .method.
  MContainer(InfoTree::ptr tree,
             on_tree_grafted_cb_t on_tree_grafted_cb,
             on_tree_pruned_cb_t on_tree_pruned_cb);

  // ------------- use
  // return 0 if id is not found
  meth_id_t get_id(const std::string& id) const;
  std::string get_name(meth_id_t id) const;
  std::vector<std::pair<std::string, MContainer::meth_id_t>> get_ids() const;
  std::map<meth_id_t, std::string> get_names() const;

  // ----------- add/remove/update (you should prefer makers for adding)

  bool remove(meth_id_t meth_id);
  bool enable(meth_id_t meth_id);
  bool disable(meth_id_t meth_id, const std::string& why);

  // invoke

  template <typename M>
  decltype(auto) invoke(meth_id_t id, typename method_trait<M>::args_t&& t) const {
    const auto& meth_it = meths_.find(id);
    return static_cast<Method2<M>*>(meth_it->second.get())->invoke_from_tuple(t);
  }

  // make

  template <typename M>
  meth_id_t make_method(const std::string& strid, InfoTree::ptr tree, M&& meth) {
    meths_.emplace(std::make_pair(++counter_, std::make_unique<Method2<M>>(std::forward<M>(meth))));
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
  CounterMap suborders_{};
};

}  // namespace switcher
#endif
