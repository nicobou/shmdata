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

template <class T>
using StorageType = typename std::decay<T>::type; 

struct AnyValueBase
{
  //AnyValueBase (const AnyValueBase &) = delete;
  virtual ~AnyValueBase () {}
  virtual AnyValueBase* clone () const = 0;
  virtual std::string to_string () const = 0;
};

template<typename T>
struct AnyValueDerived : AnyValueBase
{
  template<typename U> 
    AnyValueDerived (U &&value) : 
  value_ (std::forward<U> (value)) 
    {}
  T value_;
  AnyValueBase* clone () const {return new AnyValueDerived<T> (value_);}
  std::string to_string () const 
    {
      std::stringstream ss;
      ss << value_;
      return ss.str ();
    } 
};

template<>
struct AnyValueDerived <std::nullptr_t> : AnyValueBase
{
  template<typename U> 
    AnyValueDerived (U &&value) : 
  value_ (std::forward<U> (value)) 
    {}
  std::nullptr_t value_;
  AnyValueBase* clone () const {return new AnyValueDerived<std::nullptr_t> (value_);}
  std::string to_string () const {return std::string ("null");} 
};


struct Any
{
  bool is_null () const { return !ptr_; }
  bool not_null () const { return ptr_; }

  template<typename U> 
  Any (U&& value)
  : ptr_ (new AnyValueDerived<StorageType<U>> (std::forward<U> (value)))
  {}

  template<class U> 
  bool 
  is () const
  {
    typedef StorageType<U> T;
    auto derived = dynamic_cast<AnyValueDerived<T>*> (ptr_);
    return derived;
  }

  template<class U>
  StorageType<U>& 
  as ()
  {
    typedef StorageType<U> T;
    auto derived = dynamic_cast<AnyValueDerived<T>*> (ptr_);
    if (!derived)
      throw std::bad_cast ();
    return derived->value_;
  }

  template<class U>
  operator U ()
  {
    return as<StorageType<U>> ();
  }
  
  Any ()
  : ptr_ (nullptr)
  {}
  
  Any (Any &that)
  : ptr_ (that.clone ())
  {}
  
  Any (Any &&that)
  : ptr_ (that.ptr_)
  {
    that.ptr_ = nullptr;
  }

  Any (const Any &that)
  : ptr_ (that.clone ())
  {}

  Any (const Any&& that)
  : ptr_ (that.clone ())
  {}

  Any & 
  operator= (const Any &a)
  {
    if (ptr_ == a.ptr_)
      return *this;
    auto old_ptr = ptr_;
    ptr_ = a.clone ();
    if (old_ptr)
      delete old_ptr;
    return *this;
  }
    
    Any & 
    operator= (Any &&a)
  {
    if (ptr_ == a.ptr_)
      return *this;
    std::swap (ptr_, a.ptr_);
    return *this;
  }
    
      ~Any ()
  {
    if (ptr_)
      delete ptr_;
  }

  static std::string to_string (const Any &any)
  {
    std::stringstream ss;
    ss << any;
    return ss.str ();
  }
  
private:
  AnyValueBase* 
  clone () const
  {
    if (ptr_)
      return ptr_->clone ();
    else
      return nullptr;
  }

  AnyValueBase* ptr_;
  friend std::ostream &operator<< (std::ostream &os, const Any &any)
  {
    if (any.ptr_)
      os << any.ptr_->to_string ();
    else
      os << "null";
    return os;
  }
};

//this is for Any of complex value, where default serilization will 
//be implemented as follow
//

template <typename T>
struct DefaultSerializable {
  virtual ~DefaultSerializable() {};
  template <typename U>
  friend std::ostream &operator<< (std::ostream &os, const DefaultSerializable<U> &)
  {
    os << "not serializable";
    return os;
  }
};

#endif // ifndef
