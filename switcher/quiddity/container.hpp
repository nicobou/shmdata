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

#ifndef __SWITCHER_QUIDDITY_CONTAINER_H__
#define __SWITCHER_QUIDDITY_CONTAINER_H__

#include <memory>
#include <thread>
#include <unordered_map>

#include "../infotree/information-tree.hpp"
#include "../infotree/json-serializer.hpp"
#include "../logger/logged.hpp"
#include "../utils/ids.hpp"
#include "./config.hpp"
#include "./documentation-registry.hpp"
#include "./factory.hpp"
#include "./qrox.hpp"
#include "./quid-id-t.hpp"
#include "./quiddity.hpp"

namespace switcher {
class Switcher;
namespace quiddity {
class Bundle;

class Container : public log::Logged {
  friend class Bundle;

 public:
  using ptr = std::shared_ptr<Container>;
  using OnCreateRemoveCb = std::function<void(qid_t)>;

  static Container::ptr make_container(Switcher* switcher,
                                       quiddity::Factory* factory,
                                       log::Base* log);
  Container() = delete;
  virtual ~Container() = default;
  Container(const Container&) = delete;
  Container& operator=(const Container&) = delete;

  // infos
  InfoTree::ptr get_quiddity_description(qid_t id);
  InfoTree::ptr get_quiddities_description();
  qid_t get_id(const std::string& nickname) const;
  std::string get_nickname(qid_t id) const;
  std::vector<std::string> get_nicknames() const;
  std::vector<qid_t> get_ids() const;

  // **** creation/remove/get and notification
  Qrox create(const std::string& quiddity_kind,
              const std::string& nickname,
              InfoTree::ptrc override_config);
  Qrox quiet_create(const std::string& quiddity_kind,
                    const std::string& nickname,
                    InfoTree::ptrc tree);

  /**
  * @brief Send confirmation signal for quiddity creation.
  *        Use after a Container::quiet_create call.
  *
  * @param quid The quiddity instance returned by Container::quiet_create.
  */
  void notify_quiddity_created(Quiddity *quid);

  BoolLog remove(qid_t id);
  BoolLog quiet_remove(qid_t id);
  std::shared_ptr<Quiddity> get_quiddity(qid_t id);
  Qrox get_qrox(qid_t id);
  unsigned int register_creation_cb(OnCreateRemoveCb cb);
  unsigned int register_removal_cb(OnCreateRemoveCb cb);
  void unregister_creation_cb(unsigned int id);
  void unregister_removal_cb(unsigned int id);
  void reset_create_remove_cb();

  // information tree
  Forward_consultable_from_associative_container(
      Container,               // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      qid_t,                   // key type for accessor
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      tree,                    // method used by quiddities to access the consultable
      use_tree);               // public forwarding method

  Forward_delegate_from_associative_container(
      Container,               // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      qid_t,                   // key type for accessor
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      user_data,               // method used by quiddities to access the consultable
      user_data);              // public forwarding method

  // **** properties
  Forward_consultable_from_associative_container(
      Container,               // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      qid_t,                   // accessor key type
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      prop,                    // method used by quiddities to access the consultable
      props);                  // public forwarding method

  // **** methods
  Forward_consultable_from_associative_container(
      Container,               // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      qid_t,                   // accessor key type
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      meth,                    // method used by quiddities to access the consultable
      meths);                  // public forwarding method

  // **** signals
  Forward_consultable_from_associative_container(
      Container,               // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      qid_t,                   // accessor key type
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      sig,                     // method used by quiddities to access the consultable
      sigs);                   // public forwarding method

  Switcher* get_switcher() { return switcher_; };

 private:
  Container(Switcher* switcher, quiddity::Factory* factory, log::Base* log);

  // forwarding accessor and return constructor on error
  std::pair<bool, Quiddity*> find_quiddity(qid_t id) const {
    auto it = quiddities_.find(id);
    if (quiddities_.end() == it) {
      return std::make_pair(false, nullptr);
    }
    return std::make_pair(true, it->second.get());
  };
  // construct result to pass when element has not been found:
  template <typename ReturnType>
  ReturnType construct_error_return(qid_t id) const {
    warning("quiddity % not found", std::to_string(id));
    return ReturnType();
  }

  quiddity::Factory* factory_;
  std::map<unsigned int, OnCreateRemoveCb> on_created_cbs_{};
  std::map<unsigned int, OnCreateRemoveCb> on_removed_cbs_{};
  CounterMap counters_{};
  std::weak_ptr<Container> me_{};
  Switcher* switcher_;
  Ids ids_{};
  std::unordered_map<qid_t, std::shared_ptr<Quiddity>> quiddities_{};
};

}  // namespace quiddity
}  // namespace switcher
#endif

