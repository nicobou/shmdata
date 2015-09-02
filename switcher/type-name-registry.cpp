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

#include "./type-name-registry.hpp"

namespace switcher {

#define REGISTER_TYPE(X)                        \
  {typeid(X).hash_code(), #X}

#define REGISTER_TYPE2(X, Y)                    \
  {typeid(X).hash_code(), #Y}

TypeNameRegistry::registry_t TypeNameRegistry::type_name_registry_ = {
  REGISTER_TYPE(double),
  REGISTER_TYPE(float),
  REGISTER_TYPE(long double),
  
  REGISTER_TYPE(bool),

  REGISTER_TYPE(int),
  REGISTER_TYPE(short),
  REGISTER_TYPE(long),
  REGISTER_TYPE(long long),
  REGISTER_TYPE(unsigned short),
  REGISTER_TYPE(unsigned int),
  REGISTER_TYPE(unsigned long),
  REGISTER_TYPE(unsigned long long),

  REGISTER_TYPE(char),
  REGISTER_TYPE(char16_t),
  REGISTER_TYPE(char32_t),
  REGISTER_TYPE(wchar_t),

  REGISTER_TYPE2(std::string, string),
  REGISTER_TYPE2(const char *, string),
};

}  // namespace switcher
