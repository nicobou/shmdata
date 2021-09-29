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

/**
 * the Abstract factory class
 */

#ifndef __SWITCHER_ABSTRACT_FACTORY_H__
#define __SWITCHER_ABSTRACT_FACTORY_H__

#include <map>
#include <memory>
#include <vector>
#include "./creator.hpp"

namespace switcher {
template <class T, class Key, typename... ATs>
class AbstractFactory {
 public:
  AbstractFactory();
  ~AbstractFactory();

  template <class U>
  void register_kind(Key Id);
  void register_kind_with_custom_factory(Key Id,
                                         T* (*custom_create)(ATs...),
                                         void (*custom_destroy)(T*));
  bool unregister_kind(Key Id);
  std::vector<Key> get_keys() const;
  std::shared_ptr<T> create(Key Id, ATs... args);
  bool key_exists(Key Id) const;

 private:
  std::map<Key, Creator<T, ATs...>*> constructor_map_;
  std::map<Key, void (*)(T*)> destructor_map_;
};

}  // namespace switcher
#include "./abstract-factory_spec.hpp"
#endif
