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
#include <mutex>
#include <tuple>
#include <unordered_map>
#include "./information-tree.hpp"
#include "./property-internal-types.hpp"
#include "./property-specification.hpp"
#include "./serialize-string.hpp"

namespace switcher {
class PContainer;  // property container

class PropertyBase {
  friend class PContainer;

 public:
  using register_id_t = prop::register_id_t;
  using notify_cb_t = prop::notify_cb_t;
  using prop_id_t = prop::prop_id_t;
  PropertyBase() = delete;
  virtual ~PropertyBase() = default;
  PropertyBase(size_t type_hash);

  virtual InfoTree::ptr get_spec() = 0;
  virtual void update_value_in_spec() = 0;
  virtual bool set_str(const std::string& val, bool do_notify = true) const = 0;
  virtual std::string get_str() const = 0;
  virtual std::unique_lock<std::mutex> get_lock() = 0;
  virtual bool set_to_current() = 0;

  prop_id_t get_id() const;
  register_id_t subscribe(notify_cb_t fun) const;
  bool unsubscribe(register_id_t rid) const;
  size_t get_type_id_hash() const;
  void notify() const;
  // get/set notify cbs
  std::map<register_id_t, notify_cb_t> get_notify_cbs() const;
  void set_notify_cbs(std::map<register_id_t, notify_cb_t> cbs);
  std::string get_warning() const { return warning_; }

 protected:
  mutable std::string warning_{};

 private:
  size_t type_hash_;
  mutable register_id_t counter_{0};
  mutable std::map<register_id_t, notify_cb_t> to_notify_{};
  // id is given by other class but saved here in order to avoid
  // save it along with the Property instance
  prop_id_t id_{0};
  // following is for use by friend PContainer:
  void set_id(prop_id_t id);
  std::vector<register_id_t> get_register_ids() const;
};

template <typename V,
          typename W = V>  // readonly when set_ initialized with nullptr
class Property : public PropertyBase {
 public:
  using get_cb_t = typename prop::get_t<W>;
  using set_cb_t = typename prop::set_t<W>;

  template <typename... SpecArgs>
  Property(set_cb_t set, get_cb_t get, SpecArgs... args)
      : PropertyBase(typeid(W).hash_code()),
        doc_({static_cast<bool>(set), std::forward<SpecArgs>(args)...}),
        set_(set),
        get_(get) {}

  template <typename U = V, typename std::enable_if<std::is_same<U, Group>::value>::type* = nullptr>
  Property(const std::string& label, const std::string& description)
      : PropertyBase(typeid(Group).hash_code()),
        doc_(std::forward<const std::string&>(label),
             std::forward<const std::string&>(description)),
        set_(nullptr),
        get_(nullptr) {  // never called
  }

  bool set(const W& val, bool do_notify = true) const {
    if (nullptr == set_) {  // read only
      warning_ = "set is unavailable for read-only properties";
      return false;
    }
    if (!doc_.is_valid(val)) {  // out of range
      return false;
    }
    {
      std::unique_lock<std::mutex> lock(ts_);
      if (!set_(val))  // implementation
        return false;
    }
    if (do_notify) {
      notify();
    }
    return true;
  }

  W get() const {
    std::unique_lock<std::mutex> lock(ts_);
    return get_();
  }

  bool set_to_current() { return set(get()); }

  bool set_str(const std::string& val, bool do_notify = true) const {
    auto deserialized = deserialize::apply<W>(val);
    if (!deserialized.first) {
      warning_ = std::string("set_str failed to deserialize following string: ") + val;
      return false;
    }
    return set(std::move(deserialized.second), do_notify);
  }

  std::string get_str() const { return get_ ? serialize::apply<W>(get_()) : std::string(); }

  InfoTree::ptr get_spec() final { return doc_.get_spec(); }

  void update_value_in_spec() final {
    if (nullptr != get_) doc_.update_current_value(get());
  }

  std::unique_lock<std::mutex> get_lock() { return std::unique_lock<std::mutex>(ts_); }

 private:
  PropertySpecification<V, W> doc_;
  set_cb_t set_;
  get_cb_t get_;
  mutable std::mutex ts_{};
};

}  // namespace switcher
#endif
