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

enum AnyCategory { OTHER,
                   BOOL,
                   INTEGRAL,
                   FLOATING_POINT,
                   NONE };

enum AnyArithmeticType{ NOT_DEFINED,
                     INT,
                     SHORT,
                     LONG,
                     LONG_LONG,
                     UNSIGNED_INT,
                     UNSIGNED_SHORT,
                     UNSIGNED_LONG,
                     UNSIGNED_LONG_LONG,
                     DOUBLE,
                     FLOAT,
                     LONG_DOUBLE };

struct AnyValueBase {
  // AnyValueBase (const AnyValueBase &) = delete;
  virtual ~AnyValueBase() {}
  virtual AnyValueBase *clone() const = 0;
  virtual std::string to_string() const = 0;
  AnyCategory category_{AnyCategory::NONE};
  AnyArithmeticType arithmetic_type_{AnyArithmeticType::NOT_DEFINED};
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
    if (std::is_same<U,int>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::INT;
    else if (std::is_same<U,short>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::SHORT;
    else if (std::is_same<U,long>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::LONG;
    else if (std::is_same<U,long long>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::LONG_LONG;
    else if (std::is_same<U,unsigned int>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::UNSIGNED_INT;
    else if (std::is_same<U,unsigned short>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::UNSIGNED_SHORT;
    else if (std::is_same<U,unsigned long>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::UNSIGNED_LONG;
    else if (std::is_same<U,unsigned long long>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::UNSIGNED_LONG_LONG;
  }

  // floating point ctor
  template<typename U> Any(U && value,
                           typename std::enable_if<
                           !std::is_same<U, bool>::value && 
                           std::is_floating_point<U>::value 
                           >::type* = nullptr):
      ptr_(new AnyValueDerived<StorageType<U>> (std::forward<U>(value))) {
    ptr_->category_ = AnyCategory::FLOATING_POINT;
    if (std::is_same<U, float>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::FLOAT;
    else if (std::is_same<U, double>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::DOUBLE;
    else if (std::is_same<U, long double>::value)
      ptr_->arithmetic_type_ = AnyArithmeticType::LONG_DOUBLE;
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
  
  template<class U,
           typename std::enable_if<
             !std::is_arithmetic<U>::value
             >::type* = nullptr>
      StorageType<U> copy_as() const {
    if (AnyCategory::OTHER != ptr_->category_)
      std::cerr << "error copying back any value (is not other)" << std::endl;
    using T = StorageType<U>;
    auto derived = dynamic_cast<AnyValueDerived<T>*>(ptr_);
    if (!derived)
      return U();
    StorageType<U> res = derived->value_;
    return res;
  }

  template<typename U,
           typename std::enable_if<
             std::is_same<U, bool>::value
             >::type* = nullptr>
  U copy_as() const {
    if (AnyCategory::BOOL != ptr_->category_){
      std::cerr << "error copying back any value (is not other)" << std::endl;
      return false;
    }
    return static_cast<AnyValueDerived<bool>*>(ptr_)->value_;
  }

  template<class U,
           typename std::enable_if<
             std::is_integral<U>::value && !std::is_same<U, bool>::value
             >::type* = nullptr>
      StorageType<U> copy_as() const {
    if (AnyCategory::INTEGRAL != ptr_->category_){
      std::cerr << "error copying back any value (is not integral)" << std::endl;
      return 0;
    }
    if (AnyArithmeticType::INT == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<int>*>(ptr_)->value_; 
    else if (AnyArithmeticType::SHORT == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<short>*>(ptr_)->value_; 
    else if (AnyArithmeticType::LONG == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<long>*>(ptr_)->value_; 
    else if (AnyArithmeticType::LONG_LONG == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<long long>*>(ptr_)->value_; 
    else if (AnyArithmeticType::UNSIGNED_INT == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<unsigned int>*>(ptr_)->value_; 
    else if (AnyArithmeticType::UNSIGNED_SHORT == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<unsigned short>*>(ptr_)->value_; 
    else if (AnyArithmeticType::UNSIGNED_LONG == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<unsigned long>*>(ptr_)->value_; 
    else if (AnyArithmeticType::UNSIGNED_LONG_LONG == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<unsigned long long>*>(ptr_)->value_; 
    std::cerr << "bug in copy_as for integral" << std::endl;
    return 0;
  }

    template<class U,
           typename std::enable_if<
             std::is_floating_point<U>::value && !std::is_same<U, bool>::value
             >::type* = nullptr>
      StorageType<U> copy_as() const {
    if (AnyCategory::FLOATING_POINT != ptr_->category_){
      std::cerr << "error copying back any value (is not floating point)" << std::endl;
      return 0;
    }
    if (AnyArithmeticType::FLOAT == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<float>*>(ptr_)->value_; 
    else if (AnyArithmeticType::DOUBLE == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<double>*>(ptr_)->value_; 
    else if (AnyArithmeticType::LONG_DOUBLE == ptr_->arithmetic_type_)
      return static_cast<AnyValueDerived<long double>*>(ptr_)->value_; 
    std::cerr << "bug in copy_as for floating point" << std::endl;
    return 0.f;
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
