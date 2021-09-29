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

// this file is included from abstract-factory.hpp
// this separation is done in order to make abstract-factory.hpp easier to read

namespace switcher {
template <typename T, typename Key, typename... ATs>
template <class U>
void AbstractFactory<T, Key, ATs...>::register_kind(Key Id) {
  constructor_map_[Id] = (Creator<T, ATs...>*)new DerivedCreator<U, ATs...>();
}

template <typename T, typename Key, typename... ATs>
void AbstractFactory<T, Key, ATs...>::register_kind_with_custom_factory(
    Key Id, T* (*custom_create)(ATs...), void (*custom_destroy)(T*)) {
  CustomDerivedCreator<T, ATs...>* creator = new CustomDerivedCreator<T, ATs...>();
  creator->custom_create_ = custom_create;
  constructor_map_[Id] = (Creator<T, ATs...>*)creator;
  destructor_map_[Id] = custom_destroy;
}

template <typename T, typename Key, typename... ATs>
std::vector<Key> AbstractFactory<T, Key, ATs...>::get_keys() const {
  std::vector<Key> constructor_names;
  for (auto& it : constructor_map_) constructor_names.push_back(it.first);
  return constructor_names;
}

template <typename T, typename Key, typename... ATs>
bool AbstractFactory<T, Key, ATs...>::key_exists(Key Id) const {
  return (constructor_map_.find(Id) != constructor_map_.end());
}

template <typename T, typename Key, typename... ATs>
bool AbstractFactory<T, Key, ATs...>::unregister_kind(Key Id) {
  auto constructor_it = constructor_map_.find(Id);
  if (constructor_it == constructor_map_.end())
    return false;
  else
    delete constructor_it->second;
  constructor_map_.erase(constructor_it);
  destructor_map_.erase(Id);
  return true;
}

template <typename T, typename Key, typename... ATs>
std::shared_ptr<T> AbstractFactory<T, Key, ATs...>::create(Key Id, ATs... args) {
  std::shared_ptr<T> pointer;
  auto constructor_it = constructor_map_.find(Id);
  auto destructor_it = destructor_map_.find(Id);
  if (constructor_it != constructor_map_.end()) {
    if (destructor_it != destructor_map_.end())
      pointer.reset(constructor_it->second->Create(std::forward<ATs>(args)...),
                    destructor_it->second);
    else
      pointer.reset(constructor_it->second->Create(std::forward<ATs>(args)...));
  }
  return pointer;
}

template <typename T, typename Key, typename... ATs>
AbstractFactory<T, Key, ATs...>::~AbstractFactory() {
  for (auto& it : constructor_map_) delete it.second;
}

template <typename T, typename Key, typename... ATs>
AbstractFactory<T, Key, ATs...>::AbstractFactory() : constructor_map_(), destructor_map_() {}

}  // namespace switcher
