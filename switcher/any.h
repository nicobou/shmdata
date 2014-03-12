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
#include <type_traits>
#include <utility>
#include <typeinfo>
#include <string>
#include <ostream>

template <class T>
using StorageType = typename std::decay<T>::type; 

struct Any
{
  bool is_null () const { return !ptr_; }
  bool not_null () const { return ptr_; }

  template<typename U> 
  Any (U&& value)
  : ptr_ (new Derived<StorageType<U>> (std::forward<U> (value)))
  {}

  template<class U> 
  bool 
  is () const
  {
    typedef StorageType<U> T;
    auto derived = dynamic_cast<Derived<T>*> (ptr_);
    return derived;
  }

  template<class U>
  StorageType<U>& 
  as ()
  {
    typedef StorageType<U> T;
    auto derived = dynamic_cast<Derived<T>*> (ptr_);
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

private:
  struct Base
  {
    //Base (const Base &) = delete;
    virtual ~Base () {}
    virtual Base* clone () const = 0;
    //virtual std::string to_string () const { return std::string ("nullptr");};
  };

  template<typename T>
  struct Derived : Base
  {
    template<typename U> 
      Derived (U &&value) : 
    value_ (std::forward<U> (value)) 
      {}

    T value_;
    
    Base* 
      clone () const 
    { 
      return new Derived<T> (value_); 
    }
    
    /* //if value is not a nullptr */
    /* typename std::enable_if<!std::is_same<T, std::nullptr_t>::value, std::string>   */
    /*   to_string () const */
    /*   { */
    /* 	//std::string res; */
    /* 	//res << value_; */
    /* 	return std::string ("not a nullptr"); */
    /* } */
   
  };

  Base* 
  clone () const
  {
    if (ptr_)
      return ptr_->clone ();
    else
      return nullptr;
  }

  Base* ptr_;
  friend std::ostream &operator<< (std::ostream &os, const Any &any);
};

std::ostream & 
operator<< (std::ostream &os, const Any &any)
{
  //os << any.ptr_->to_string ();
  return os;
}
