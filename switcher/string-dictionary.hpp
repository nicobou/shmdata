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

#ifndef __SWITCHER_STRING_DICTIONARY_H__
#define __SWITCHER_STRING_DICTIONARY_H__

#include "./quiddity.hpp"

namespace switcher {
class StringDictionary:public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(StringDictionary);
  StringDictionary(const std::string &){}
  ~StringDictionary() = default;
  StringDictionary(const StringDictionary &) = delete;
  StringDictionary &operator=(const StringDictionary &) = delete;
  bool init() final;

 private:
  // methods
  static gboolean update_entry(const gchar *name,
                               const gchar *value,
                               void *user_data);
  static gboolean remove_entry(const gchar *name,
                               void *user_data);
  static const gchar *read_entry(const gchar *name, void *user_data);
  static gboolean save(gchar *file_path, void *user_data);
  static gboolean load(gchar *file_path, void *user_data);
  gboolean load_file(const gchar *file_path);
  gboolean save_file(const gchar *file_path);
};
}  // namespace switcher
#endif
