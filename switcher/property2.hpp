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

#ifndef __SWITCHER_PROPERTY2_H__
#define __SWITCHER_PROPERTY2_H__

#include <map>
#include "./information-tree.hpp"
#include "./property-specification.hpp"

namespace switcher {
class PropertyBase{
 public:
  using register_id_t = size_t;
  using notify_cb_t = std::function<void()>;
  PropertyBase() = delete;
  virtual ~PropertyBase() = default;
  PropertyBase(size_t type_hash) :
      type_hash_(type_hash){
  }
  register_id_t subscribe(notify_cb_t fun){
    to_notify_[++counter_] = fun;
  return counter_;
  }

  virtual data::Tree::ptr get_spec() = 0;
  
 protected:
  size_t get_type_id_hash(){
  return type_hash_;
  }
  void notify(){
    for(auto &it: to_notify_)
    it.second();
  }

 private:
  size_t type_hash_;
  register_id_t counter_{0};
  std::map<register_id_t, notify_cb_t> to_notify_{};
};

template<class V>  // readonly when set_ initialized with nullptr
class Property2: public PropertyBase{
 public:
  using get_cb_t = std::function<V()>;
  using set_cb_t = std::function<bool(const V &)>;
  template <typename ...SpecArgs>
  Property2(set_cb_t set,
            get_cb_t get,
            SpecArgs ...args):
      PropertyBase(typeid(V).hash_code()),
      doc_({args...}),
      set_(set),
      get_(get){
  }

  bool set(const V &val, bool do_notify = true){
    if (nullptr == set_)
      return false;  // read only
    if (!set_(std::forward<const V &>(val)))
      return false;
    if (do_notify)
      notify();
    return true;
  }

  V get(){
    return get_();
  }
  
  bool set_str(const std::string &val, bool do_notify = true){
    if (nullptr == set_)
      return false;  // read only
    std::istringstream iss (val);
    V tmp;
    iss >> tmp;
    bool res = set_(tmp);
    if(res && do_notify)
      notify();
    return res;
  }
  
  std::string get_str(){
    std::ostringstream oss;
    oss << get_();
    return oss.str();
  }

  data::Tree::ptr get_spec() final {return doc_.get_spec();};
  
 private:
  PropertySpecification<V> doc_;
  set_cb_t set_;
  get_cb_t get_;
};

}  // namespace switcher
#endif
