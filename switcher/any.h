#include <type_traits>
#include <utility>
#include <typeinfo>
#include <string>

template <class T>
using StorageType = typename std::decay<T>::type; 

struct Any
{
  bool is_null () const { return !ptr_; }
  bool not_null () const { return ptr_; }

  template<typename U> Any (U&& value)
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

  Any& 
  operator= (const Any& a)
  {
    if (ptr_ == a.ptr_)
      return *this;
    auto old_ptr = ptr_;
    ptr_ = a.clone ();
    if (old_ptr)
      delete old_ptr;
    return *this;
  }
    
  Any& 
  operator= (Any&& a)
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
};
