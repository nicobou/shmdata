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

#include "./custom-property-helper.hpp"

namespace switcher {
CustomPropertyHelper::CustomPropertyHelper() {
  gobject_.reset(new GObjectWrapper());
}

GObject *CustomPropertyHelper::get_gobject() {
  return gobject_->get_gobject();
}

GParamSpec *CustomPropertyHelper::make_string_property(const gchar *
                                                       nickname,
                                                       const gchar *
                                                       description,
                                                       const gchar *
                                                       default_value,
                                                       GParamFlags
                                                       read_write_flags,
                                                       set_string_method
                                                       set_method,
                                                       get_string_method
                                                       get_method,
                                                       void *user_data) {
  GParamSpec *pspec = GObjectWrapper::make_string_property(nickname,
                                                           description,
                                                           default_value,
                                                           read_write_flags,
                                                           set_by_gvalue,
                                                           get_by_gvalue);

  make_user_method(nickname,
                   pspec,
                   (void (*)(void)) set_method,
                   (void (*)(void)) get_method, user_data);
  return pspec;
}

GParamSpec *CustomPropertyHelper::make_boolean_property(const gchar *
                                                        nickname,
                                                        const gchar *
                                                        description,
                                                        gboolean
                                                        default_value,
                                                        GParamFlags
                                                        read_write_flags,
                                                        set_boolean_method
                                                        set_method,
                                                        get_boolean_method
                                                        get_method,
                                                        void *user_data) {
  GParamSpec *pspec = GObjectWrapper::make_boolean_property(nickname,
                                                            description,
                                                            default_value,
                                                            read_write_flags,
                                                            set_by_gvalue,
                                                            get_by_gvalue);

  make_user_method(nickname,
                   pspec,
                   (void (*)(void)) set_method,
                   (void (*)(void)) get_method, user_data);
  return pspec;
}

GParamSpec *CustomPropertyHelper::make_int_property(const gchar *nickname,
                                                    const gchar *
                                                    description,
                                                    gint min_value,
                                                    gint max_value,
                                                    gint default_value,
                                                    GParamFlags
                                                    read_write_flags,
                                                    set_int_method
                                                    set_method,
                                                    get_int_method
                                                    get_method,
                                                    void *user_data) {
  GParamSpec *pspec = GObjectWrapper::make_int_property(nickname,
                                                        description,
                                                        min_value,
                                                        max_value,
                                                        default_value,
                                                        read_write_flags,
                                                        set_by_gvalue,
                                                        get_by_gvalue);

  make_user_method(nickname,
                   pspec,
                   (void (*)(void)) set_method,
                   (void (*)(void)) get_method, user_data);
  return pspec;
}

GParamSpec *CustomPropertyHelper::make_double_property(const gchar *
                                                       nickname,
                                                       const gchar *
                                                       description,
                                                       gdouble min_value,
                                                       gdouble max_value,
                                                       gdouble
                                                       default_value,
                                                       GParamFlags
                                                       read_write_flags,
                                                       set_double_method
                                                       set_method,
                                                       get_double_method
                                                       get_method,
                                                       void *user_data) {
  GParamSpec *pspec = GObjectWrapper::make_double_property(nickname,
                                                           description,
                                                           min_value,
                                                           max_value,
                                                           default_value,
                                                           read_write_flags,
                                                           set_by_gvalue,
                                                           get_by_gvalue);

  make_user_method(nickname,
                   pspec,
                   (void (*)(void)) set_method,
                   (void (*)(void)) get_method, user_data);
  return pspec;
}

GParamSpec *CustomPropertyHelper::make_enum_property(const gchar *
                                                     nickname,
                                                     const gchar *
                                                     description,
                                                     const gint
                                                     default_value,
                                                     const GEnumValue *
                                                     custom_enum,
                                                     GParamFlags
                                                     read_write_flags,
                                                     set_enum_method
                                                     set_method,
                                                     get_enum_method
                                                     get_method,
                                                     void *user_data) {
  GParamSpec *pspec = GObjectWrapper::make_enum_property(nickname,
                                                         description,
                                                         default_value,
                                                         custom_enum,
                                                         read_write_flags,
                                                         set_by_gvalue,
                                                         get_by_gvalue);

  make_user_method(nickname,
                   pspec,
                   (void (*)(void)) set_method,
                   (void (*)(void)) get_method, user_data);
  return pspec;
}

void
CustomPropertyHelper::make_user_method(const gchar *nickname,
                                       GParamSpec *pspec,
                                       void(*set_method)(void),
                                       void(*get_method)(void),
                                       void *user_data) {
  std::shared_ptr<UserMethod> user_method(new UserMethod());
  user_method->set = (void (*)(void)) set_method;
  user_method->get = (void (*)(void)) get_method;
  user_method->user_data = user_data;
  user_method->gobject = get_gobject();
  user_method->pspec = pspec;
  user_methods_.push_back(user_method);
  gobject_->property_set_user_data(nickname, user_method.get());
}

bool CustomPropertyHelper::get_by_gvalue(GValue *value, void *user_data) {
  UserMethod *user_method = static_cast<UserMethod *>(user_data);

  if (G_VALUE_TYPE(value) == G_TYPE_STRING) {
    const gchar *val =
        ((get_string_method) user_method->get) (user_method->user_data);
    g_value_set_string(value, val);
  } else if (G_VALUE_TYPE(value) == G_TYPE_BOOLEAN) {
    gboolean val =
        ((get_boolean_method) user_method->get) (user_method->user_data);
    g_value_set_boolean(value, val);
  } else if (G_VALUE_TYPE(value) == G_TYPE_INT) {
    gint val = ((get_int_method) user_method->get) (user_method->user_data);
    g_value_set_int(value, val);
  } else if (G_TYPE_IS_ENUM(G_VALUE_TYPE(value))) {
    gint val =
        ((get_enum_method) user_method->get) (user_method->user_data);
    g_value_set_enum(value, val);
  } else if (G_VALUE_TYPE(value) == G_TYPE_DOUBLE) {
    gdouble val =
        ((get_double_method) user_method->get) (user_method->user_data);
    g_value_set_double(value, val);
  } else {
    g_debug("CustomPropertyHelper: unknown type");
  }
  return TRUE;
}

bool
CustomPropertyHelper::set_by_gvalue(const GValue *value,
                                    void *user_data) {
  UserMethod *user_method = static_cast<UserMethod *>(user_data);

  if (G_VALUE_TYPE(value) == G_TYPE_STRING) {
    ((set_string_method) user_method->set) (g_value_get_string(value),
                                            user_method->user_data);
  } else if (G_VALUE_TYPE(value) == G_TYPE_BOOLEAN) {
    ((set_boolean_method) user_method->set) (g_value_get_boolean(value),
                                             user_method->user_data);
  } else if (G_VALUE_TYPE(value) == G_TYPE_INT) {
    ((set_int_method) user_method->set) (g_value_get_int(value),
                                         user_method->user_data);
  } else if (G_TYPE_IS_ENUM(G_VALUE_TYPE(value))) {
    ((set_enum_method) user_method->set) (g_value_get_enum(value),
                                          user_method->user_data);
  } else if (G_VALUE_TYPE(value) == G_TYPE_DOUBLE) {
    ((set_double_method) user_method->set) (g_value_get_double(value),
                                            user_method->user_data);
  } else {
    g_debug("CustomPropertyHelper: %s is unhandled type",
            g_type_name(G_VALUE_TYPE(value)));
    return FALSE;
  }
  GObjectWrapper::notify_property_changed(user_method->gobject,
                                          user_method->pspec);
  return TRUE;
}

bool CustomPropertyHelper::notify_property_changed(GParamSpec *pspec) {
  return GObjectWrapper::notify_property_changed(gobject_->get_gobject(),
                                                 pspec);
}

bool CustomPropertyHelper::is_property_nickname_taken(std::string nickname) {
  return gobject_->is_property_nickname_taken(nickname);
}
}
