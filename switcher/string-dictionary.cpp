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

#include <string.h>
#include <gio/gio.h>
//#include <iostream>
#include "./string-dictionary.hpp"
#include "./information-tree-basic-serializer.hpp"
#include "./scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    StringDictionary,
    "dico",
    "Dictionary",
    "utils",
    "",
    "Dictionary of string key/values accessible through properties",
    "LGPL",
    "Nicolas Bouillot");

bool StringDictionary::init() {
  install_method("Update An Entry",
                 "update",
                 "update an existing entry",
                 "success or failure",
                 Method::make_arg_description("Entry Name (Key)",
                                              "name",
                                              "string",
                                              "Entry value",
                                              "value",
                                              "string",
                                              nullptr),
                 (Method::method_ptr) &update_entry,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   G_TYPE_STRING,
                                                   nullptr),
                 this);
  
  install_method("Read Entry",
                 "read",
                 "read the entry",
                 "the entry value",
                 Method::make_arg_description("Entry Name",
                                              "name",
                                              "string",
                                              nullptr),
                 (Method::method_ptr) &read_entry,
                 G_TYPE_STRING,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr),
                 this);

  install_method("Remove An Entry",
                 "remove",
                 "remove the entry",
                 "success or failure",
                 Method::make_arg_description("Entry Name",
                                              "name",
                                              "string",
                                              nullptr),
                 (Method::method_ptr) &remove_entry,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr),
                 this);

  install_method("Save To A File",
                 "save",
                 "save dictionary to a file",
                 "success or failure",
                 Method::make_arg_description("File path",
                                              "path",
                                              "string",
                                              nullptr),
                 (Method::method_ptr) &save,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr),
                 this);

  install_method("Load From File",
                 "load",
                 "Load dictionary from a file",
                 "success or failure",
                 Method::make_arg_description("File path",
                                              "path",
                                              "string",
                                              nullptr),
                 (Method::method_ptr) &load,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr),
                 this);

  return true;
}

gboolean
StringDictionary::update_entry(const gchar *name,
                               const gchar *value,
                               void *user_data) {
  StringDictionary *context = static_cast<StringDictionary *>(user_data);
  if (context->graft_tree(std::string(".dico.") + data::Tree::escape_dots(name),
                          data::Tree::make(std::string(value))))
    return TRUE;
  g_warning("cannot update entry .dico.%s", name);
  return FALSE;
}

gboolean
StringDictionary::remove_entry(const gchar *name, void *user_data)
{
  StringDictionary *context = static_cast<StringDictionary *>(user_data);
  if (context->prune_tree(std::string(".dico.") + data::Tree::escape_dots(name)))
    return TRUE;
  g_warning("cannot remove entry .dico.%s", name);
  return FALSE;
}

const gchar *
StringDictionary::read_entry(const gchar *name, void *user_data)
{
  StringDictionary *context = static_cast<StringDictionary *>(user_data);
  std::string val = context->
      tree<const Any &, const std::string &>(
          &data::Tree::branch_read_data,
          std::string("dico.") + data::Tree::escape_dots(name)).copy_as<std::string>();
  return g_strdup(val.c_str());  // FIXME make method class not requiring g_strdup
}

gboolean StringDictionary::save_file(const gchar *file_path) {
  GFile *file = g_file_new_for_commandline_arg(file_path);
  On_scope_exit{g_object_unref(file);};
  GError *error = nullptr;
  GFileOutputStream *file_stream = g_file_replace(file,
                                                  nullptr,
                                                  TRUE,  // make backup
                                                  G_FILE_CREATE_NONE,
                                                  nullptr,
                                                  &error);
  On_scope_exit{g_object_unref(file_stream);};
  
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return FALSE;
  }

  std::string dico = invoke_info_tree<std::string>(&data::BasicSerializer::serialize);

  g_output_stream_write((GOutputStream *) file_stream,
                        dico.c_str(),
                        sizeof(char) * dico.size(),
                        nullptr,
                        &error);
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return FALSE;
  }

  g_output_stream_close((GOutputStream *) file_stream, nullptr, &error);
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return FALSE;
  }

  return TRUE;
}

gboolean StringDictionary::save(gchar *file_path, void *user_data) {
  StringDictionary *context = static_cast<StringDictionary *>(user_data);
  return context->save_file(file_path);
}

gboolean StringDictionary::load_file(const gchar *filename) {
  gchar *contents = nullptr;
  GError *error = nullptr;
  g_file_get_contents (filename,
                       &contents,
                       nullptr,
                       &error);
  On_scope_exit{if (nullptr != contents) g_free(contents);};
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return FALSE;
  }

  data::Tree::ptr tree = data::BasicSerializer::deserialize(std::string(contents));
  // replacing the dico
  graft_tree(std::string(".dico."), tree->get(".dico"));
  return TRUE;
}

gboolean StringDictionary::load(gchar *file_path, void *user_data) {
  StringDictionary *context = static_cast<StringDictionary *>(user_data);
  return context->load_file(file_path);
}
}
