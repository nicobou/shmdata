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

#include <vector>
#include <map>
#include "./creator.hpp"

namespace switcher {
template<class T, class Key, class Doc, typename ...ATs> class AbstractFactory {
 public:
  template<class U> void register_class(Key Id, Doc doc);
  AbstractFactory();
  ~AbstractFactory();

  void register_class_with_custom_factory(Key Id,
                                          Doc doc,
                                          T *(*custom_create) (ATs...),
                                          void(*custom_destroy) (T *));
  bool unregister_class(Key Id);
  std::vector<Key> get_keys();
  std::vector<Doc> get_classes_documentation();
  Doc get_class_documentation(Key Id);
  std::shared_ptr<T> create(Key Id, ATs... args);
  bool key_exists(Key Id);

 private:
  std::map<Key, Creator<T, ATs...> *> constructor_map_;
  std::map<Key, void(*)(T *)> destructor_map_;
  std::map<Key, Doc> classes_documentation_;
};

}  // namespace switcher
#include "./abstract-factory_spec.hpp"
#endif
