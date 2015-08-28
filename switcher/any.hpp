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

#ifndef __SWITCHER_ANY_H__
#define __SWITCHER_ANY_H__

#include <type_traits>
#include <utility>
#include <typeinfo>
#include <string>
#include <ostream>
#include <sstream>
#include <iostream>  // FIXME remove that

template<class T> using StorageType = typename std::decay<T>::type;

enum AnyCategory { BOOL, INTEGRAL, FLOAT, OTHER, NONE };

struct AnyValueBase {
  // AnyValueBase (const AnyValueBase &) = delete;
  virtual ~AnyValueBase() {}
  virtual AnyValueBase *clone() const = 0;
  virtual std::string to_string() const = 0;
  AnyCategory category_{AnyCategory::NONE};
};

template<typename T> struct AnyValueDerived: AnyValueBase {
  template <typename U>
  AnyValueDerived(U &&value, AnyCategory category = AnyCategory::NONE):
      value_(std::forward<U> (value)){
    category_ = category;
  }
  T value_;
  AnyValueBase *clone() const {
    return new AnyValueDerived <T> (value_, category_);
  }
  
  std::string to_string() const {
    std::stringstream ss;
    ss << value_;
    return ss.str();    
  }
};

template <>
struct AnyValueDerived <std::nullptr_t> : AnyValueBase {
  template<typename U> AnyValueDerived(U && value):
      value_(std::forward<U> (value)) {
  }
  std::nullptr_t value_;
  AnyValueBase *clone() const {
    return new AnyValueDerived <std::nullptr_t> (value_);
  }
  std::string to_string() const {
    return std::string("null");
  }
};

struct Any {
  bool is_null() const {
    return !ptr_;
  }
  bool not_null() const {
    return ptr_;
  }

  AnyCategory get_category() const{
    return ptr_ ? ptr_->category_ : AnyCategory::NONE;
  }
  
  // default ctor
  template<typename U> Any(U &&value,
                           typename std::enable_if<
                           !std::is_arithmetic<U>::value>::type* = nullptr):
      ptr_(new AnyValueDerived<StorageType<U>> (std::forward<U>(value))) {
    ptr_->category_ = AnyCategory::OTHER;
  }

  // bool ctor
  template<typename U = bool> Any(bool &&value):
      ptr_(new AnyValueDerived<StorageType<U>> (std::forward<U>(value))){
    ptr_->category_ = AnyCategory::BOOL;
  }

  // integral ctor
  template<typename U> Any(U && value,
                           typename std::enable_if<
                           !std::is_same<U, bool>::value && 
                           std::is_integral<U>::value 
                           >::type* = nullptr):
      ptr_(new AnyValueDerived<StorageType<U>> (std::forward<U>(value))) {
     ptr_->category_ = AnyCategory::INTEGRAL;
  }

  // floating point ctor
  template<typename U> Any(U && value,
                           typename std::enable_if<
                           !std::is_same<U, bool>::value && 
                           std::is_floating_point<U>::value 
                           >::type* = nullptr):
      ptr_(new AnyValueDerived<StorageType<U>> (std::forward<U>(value))) {
    ptr_->category_ = AnyCategory::FLOAT;
  }

  Any(std::nullptr_t /*value*/):
      ptr_(nullptr) {
  }

  template<class U>
  bool is() const {
    typedef StorageType<U> T;
    auto derived = dynamic_cast<AnyValueDerived<T>*>(ptr_);
    return derived;
  }

  template<class U>
  StorageType<U> &as() const {
    using T = StorageType<U>;
    auto derived = dynamic_cast<AnyValueDerived<T>*>(ptr_);
    if (!derived)
      return *new U;
    return derived->value_;
  }
  
  template<class U>
  StorageType<U> copy_as() const {
    using T = StorageType<U>;
    auto derived = dynamic_cast<AnyValueDerived<T>*>(ptr_);
    if (!derived)
      return U();
    StorageType<U> res = derived->value_;
    return res;
  }
  
  template<class U>
  operator U() const {
    return as<StorageType<U>>();
  }

  Any(): ptr_(nullptr) {
  }

  Any(Any &that):ptr_(that.clone()) {
  }

  Any(Any &&that):ptr_(that.ptr_) {
    that.ptr_ = nullptr;
  }

  Any(const Any &that):ptr_(that.clone()) {
  }

  Any(const Any &&that):
      ptr_(that.clone()) {
  }

  Any &operator=(const Any &a) {
    if (ptr_ == a.ptr_)
      return *this;
    auto old_ptr = ptr_;
    ptr_ = a.clone();
    if (old_ptr)
      delete old_ptr;
    return *this;
  }

  Any &operator=(Any && a) {
    if (ptr_ == a.ptr_)
      return *this;
    std::swap(ptr_, a.ptr_);
    return *this;
  }

  ~Any() {
    if (ptr_)
      delete ptr_;
  }

  static std::string to_string(const Any &any) {
    std::stringstream ss;
    ss << any;
    return ss.str();
  }

 private:
  AnyValueBase *clone() const {
    if (ptr_)
      return ptr_->clone();
    else
      return nullptr;
  }
  
  AnyValueBase *ptr_;
  friend std::ostream &operator<<(std::ostream &os, const Any &any) {
    if (any.ptr_)
      os << any.ptr_->to_string();
    else
      os << "null";
    return os;
  }
};

// this is for Any of complex value, where default serilization will
// be implemented as follow
//
template<typename T> struct DefaultSerializable {
  virtual ~DefaultSerializable() {}
  template<typename U>
  friend std::ostream &operator<<(std::ostream &os, const DefaultSerializable<U> &) {
    os << "not serializable";
    return os;
  }
};

#endif
