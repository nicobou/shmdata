/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "quiddity.h"
#include "custom-property-helper.h"
#include <map>

namespace switcher
{
  class StringDictionary : public Quiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(StringDictionary);
    StringDictionary ();
    ~StringDictionary ();
    StringDictionary (const StringDictionary  &) = delete;
    StringDictionary &operator= (const StringDictionary  &) = delete;
    bool init ();

  private:
    std::map <std::string, gchar *> dico_;

    typedef struct {
      StringDictionary *string_dictionary;
      std::string entry_name;
    } PropertySetGet;
    std::map<std::string, std::shared_ptr<PropertySetGet> > set_get_contexts_;

    //property
    CustomPropertyHelper::ptr custom_props_;
    std::map<std::string, GParamSpec *> prop_specs_;
    static gchar* string_getter (void *user_data);
    static void string_setter (const gchar *value, 
			       void *user_data);

    //methods
    static gboolean create_entry (const gchar *entry_name, 
				  const gchar *description, 
				  const gchar *long_name,
				  void *user_data);
    static gboolean remove_entry (const gchar *entry_name, 
				  void *user_data);
    static gboolean save (gchar *file_path, 
			  void *user_data);
    static gboolean load (gchar *file_path, 
			  void *user_data);
    gboolean load_file (const gchar *file_path);
    gboolean save_file (const gchar *file_path);
  };
  
}  // end of namespace

#endif // ifndef
