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

#include <gst/gst.h>
#include "./gprop-to-prop.hpp"
#include "./scope-exit.hpp"

namespace switcher {
namespace GPropToProp{

std::unique_ptr<PropertyBase> to_prop(GObject *gobj, const std::string &gprop_name){
  GParamSpec *pspec =
      g_object_class_find_property(G_OBJECT_GET_CLASS(gobj), gprop_name.c_str());
  if (nullptr == pspec){
    g_warning("property %s not found for the gobject calss %s",
              gprop_name.c_str(),
              G_OBJECT_CLASS_NAME(G_OBJECT_GET_CLASS(gobj)));
    return std::unique_ptr<PropertyBase>();
  }

  GValue value = G_VALUE_INIT;
  g_value_init(&value, pspec->value_type);
  g_object_get_property(gobj, pspec->name, &value);
  std::string description = std::string(g_param_spec_get_blurb(pspec));
  bool is_writable = false;
  if (pspec->flags &G_PARAM_WRITABLE)
    is_writable = true;

  auto res = std::unique_ptr<PropertyBase>();
  switch (G_VALUE_TYPE(&value)) {
    case G_TYPE_STRING:
      res = std2::make_unique<Property2<std::string>>(
          is_writable ?
          [gobj, gprop_name](const std::string &val){
            g_object_set(gobj, gprop_name.c_str(), val.c_str(), nullptr);
            return true;
          } : static_cast<prop::set_t<std::string>>(nullptr),  // FIXME
          [gobj, gprop_name](){
            gchar *strval; On_scope_exit{g_free(strval);};
            g_object_get(gobj, gprop_name.c_str(), &strval, nullptr);
            return std::string(strval);},
          gprop_name,
          std::string(description),
          std::string(g_value_get_string(&value)));
      break;
    case G_TYPE_BOOLEAN:
      res = std2::make_unique<Property2<bool>>(
          is_writable ?
          [gobj, gprop_name](const bool &val){
            g_object_set(gobj, gprop_name.c_str(), val ? TRUE : FALSE, nullptr);
            return true;
          } : static_cast<prop::set_t<bool>>(nullptr),
          [gobj, gprop_name](){
            gboolean val;
            g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
            return val ? true : false;
          },
          gprop_name,
          std::string(description),
          g_value_get_boolean(&value) ? true : false);
      break;
    case G_TYPE_ULONG:
      {
        GParamSpecULong *pulong = G_PARAM_SPEC_ULONG(pspec);
        res = std2::make_unique<Property2<gulong>>(
            is_writable ?
            [gobj, gprop_name](const gulong &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<gulong>>(nullptr),
            [gobj, gprop_name](){
              gulong val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_ulong(&value),
            pulong->minimum,
            pulong->maximum);
      }
        break;
          case G_TYPE_LONG:
      {
        GParamSpecLong *plong = G_PARAM_SPEC_LONG(pspec);
        res = std2::make_unique<Property2<glong>>(
            is_writable ?
            [gobj, gprop_name](const glong &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<glong>>(nullptr),
            [gobj, gprop_name](){
              glong val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_long(&value),
            plong->minimum,
            plong->maximum);
      }
        break;
    case G_TYPE_UINT:
      {
        GParamSpecUInt *puint = G_PARAM_SPEC_UINT(pspec);
        res = std2::make_unique<Property2<guint>>(
            is_writable ?
            [gobj, gprop_name](const guint &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<guint>>(nullptr),
            [gobj, gprop_name](){
              guint val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_uint(&value),
            puint->minimum,
            puint->maximum);
      }
      break;
    case G_TYPE_INT:
      {
        GParamSpecInt *pint = G_PARAM_SPEC_INT(pspec);
        res = std2::make_unique<Property2<gint>>(
            is_writable ?
            [gobj, gprop_name](const gint &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<gint>>(nullptr),
            [gobj, gprop_name](){
              gint val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_int(&value),
            pint->minimum,
            pint->maximum);
      }
      break;
    case G_TYPE_UINT64:
      {
        GParamSpecUInt64 *puint64 = G_PARAM_SPEC_UINT64(pspec);
         res = std2::make_unique<Property2<guint64>>(
            is_writable ?
            [gobj, gprop_name](const guint64 &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<guint64>>(nullptr),
            [gobj, gprop_name](){
              guint64 val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_uint64(&value),
            puint64->minimum,
            puint64->maximum);
     }
      break;
    case G_TYPE_INT64:
      {
        GParamSpecInt64 *pint64 = G_PARAM_SPEC_INT64(pspec);
         res = std2::make_unique<Property2<gint64>>(
            is_writable ?
            [gobj, gprop_name](const gint64 &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<gint64>>(nullptr),
            [gobj, gprop_name](){
              gint64 val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_int64(&value),
            pint64->minimum,
            pint64->maximum);
      }
      break;
    case G_TYPE_FLOAT:
      {
        GParamSpecFloat *pfloat = G_PARAM_SPEC_FLOAT(pspec);
         res = std2::make_unique<Property2<gfloat>>(
            is_writable ?
            [gobj, gprop_name](const gfloat &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<gfloat>>(nullptr),
            [gobj, gprop_name](){
              gfloat val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_float(&value),
            pfloat->minimum,
            pfloat->maximum);
      }
      break;
    case G_TYPE_DOUBLE:
      {
        GParamSpecDouble *pdouble = G_PARAM_SPEC_DOUBLE(pspec);
         res = std2::make_unique<Property2<gdouble>>(
            is_writable ?
            [gobj, gprop_name](const gdouble &val){
              g_object_set(gobj, gprop_name.c_str(), val, nullptr);
              return true;
            } : static_cast<prop::set_t<gdouble>>(nullptr),
            [gobj, gprop_name](){
              gdouble val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return val;
            },
            gprop_name,
            std::string(description),
            g_value_get_double(&value),
            pdouble->minimum,
            pdouble->maximum);
      }
      break;
  //   default:
  //     if (pspec->value_type == GST_TYPE_CAPS) {
  //       const GstCaps *caps = gst_value_get_caps(&value);
  //       json_description_->add_string_member("type", "caps");
  //       if (!caps)
  //         json_description_->add_string_member("default value", "");
  //       else
  //         json_description_->add_string_member("default value",
  //                                              gst_caps_to_string(caps));
  //     } else if (G_IS_PARAM_SPEC_ENUM(pspec)) {
  //       GEnumValue *values;
  //       guint j = 0;
  //       gint enum_value;
  //       const gchar *value_nick = "";
  //       const gchar *value_name = "";
  //       json_description_->add_string_member("type", "enum");
  //       values =
  //           G_ENUM_CLASS(g_type_class_ref(pspec->value_type))->values;
  //       enum_value = g_value_get_enum(&value);
  //       while (values[j].value_name) {
  //         if (values[j].value == enum_value) {
  //           value_nick = values[j].value_nick;
  //           value_name = values[j].value_name;
  //         }
  //         j++;
  //       }

  //       json_description_->set_member_name("default value");
  //       json_description_->begin_object();
  //       gchar *value = g_strdup_printf("%d", enum_value);
  //       json_description_->add_string_member("value", value);
  //       g_free(value);
  //       json_description_->add_string_member("nick", value_nick);
  //       json_description_->add_string_member("name", value_name);
  //       json_description_->end_object();

  //       // g_debug ("Enum \"%s\" Default: %d, \"%s\" \"%s\"",
  //       //  g_type_name (G_VALUE_TYPE (&value)),
  //       //  enum_value,
  //       //  value_nick,
  //       //  value_name);

  //       j = 0;

  //       json_description_->set_member_name("values");
  //       json_description_->begin_array();
  //       while (values[j].value_name) {
  //         json_description_->begin_object();
  //         json_description_->add_string_member("name", values[j].value_name);
  //         json_description_->add_string_member("nick", values[j].value_nick);
  //         gchar *values_value = g_strdup_printf("%d", values[j].value);
  //         json_description_->add_string_member("value", values_value);
  //         g_free(values_value);
  //         json_description_->end_object();
  //         j++;
  //       }
  //       json_description_->end_array();

  //       /* g_type_class_unref (ec); */
  //     } else if (G_IS_PARAM_SPEC_FLAGS(pspec)) {
  //       g_debug("warning: param spec flags not handled");
  //       // GParamSpecFlags *pflags = G_PARAM_SPEC_FLAGS (pspec);
  //       // GFlagsValue *vals;
  //       // gchar *cur;

  //       // vals = pflags->flags_class->values;

  //       // cur = flags_to_string (vals, g_value_get_flags (&value));

  //       // g_debug ("%-23.23s Flags \"%s\" Default: 0x%08x, \"%s\"", "",
  //       //    g_type_name (G_VALUE_TYPE (&value)),
  //       //    g_value_get_flags (&value), cur);

  //       // while (vals[0].value_name) {
  //       //   g_debug ("");
  //       //   if (_name)
  //       //     g_debug ("%s", _name);
  //       //   g_debug ("%-23.23s    (0x%08x): %-16s - %s", "",
  //       //      vals[0].value, vals[0].value_nick, vals[0].value_name);
  //       //   ++vals;
  //       // }

  //       // g_free (cur);
  //     } else if (G_IS_PARAM_SPEC_OBJECT(pspec)) {
  //       g_debug("warning: param spec object not handled");
  //       // g_debug ("%-23.23s Object of type \"%s\"", "",
  //       //  g_type_name (pspec->value_type));
  //     } else if (G_IS_PARAM_SPEC_BOXED(pspec)) {
  //       g_debug("warning: param spec boxed not handled");
  //       // g_debug ("%-23.23s Boxed pointer of type \"%s\"", "",
  //       //  g_type_name (pspec->value_type));
  //     } else if (G_IS_PARAM_SPEC_POINTER(pspec)) {
  //       g_debug("warning: param spec pointer not handled");
  //       // if (pspec->value_type != G_TYPE_POINTER) {
  //       //   g_debug ("%-23.23s Pointer of type \"%s\".", "",
  //       //    g_type_name (pspec->value_type));
  //       // } else if (pspec->value_type == G_TYPE_VALUE_ARRAY) {
  //       // GParamSpecValueArray *pvarray = G_PARAM_SPEC_VALUE_ARRAY (pspec);
  //       // g_debug ("warning: array not handled");
  //       // if (pvarray->element_spec) {
  //       //   g_debug ("%-23.23s Array of GValues of type \"%s\"", "",
  //       //    g_type_name (pvarray->element_spec->value_type));
  //       // } else {
  //       //   g_debug ("%-23.23s Array of GValues", "");
  //       // }
  //     } else if (GST_IS_PARAM_SPEC_FRACTION(pspec)) {
  //       GstParamSpecFraction *pfraction = GST_PARAM_SPEC_FRACTION(pspec);
  //       json_description_->add_string_member("type", "fraction");
  //       gchar *minnum = g_strdup_printf("%d", pfraction->min_num);
  //       gchar *minden = g_strdup_printf("%d", pfraction->min_den);
  //       gchar *maxnum = g_strdup_printf("%d", pfraction->max_num);
  //       gchar *maxden = g_strdup_printf("%d", pfraction->max_den);
  //       gchar *defaultnum = g_strdup_printf("%d",
  //                                           gst_value_get_fraction_numerator
  //                                           (&value));
  //       gchar *defaultden = g_strdup_printf("%d",
  //                                           gst_value_get_fraction_denominator
  //                                           (&value));
  //       json_description_->add_string_member("minimum numerator", minnum);
  //       json_description_->add_string_member("maximum numerator", minden);
  //       json_description_->add_string_member("minimum denominator", maxnum);
  //       json_description_->add_string_member("maximum denominator", maxden);
  //       json_description_->add_string_member("default numerator", defaultnum);
  //       json_description_->add_string_member("default denominator",
  //                                            defaultden);
  //       g_free(minnum);
  //       g_free(minden);
  //       g_free(maxnum);
  //       g_free(maxden);
  //       g_free(defaultnum);
  //       g_free(defaultden);
  //       // g_debug ("Range: %d/%d - %d/%d Default: %d/%d ",
  //       //  pfraction->min_num, pfraction->min_den,
  //       //  pfraction->max_num, pfraction->max_den,
  //       //  gst_value_get_fraction_numerator (&value),
  //       //  gst_value_get_fraction_denominator (&value));
  //     } else {
  //       g_warning("warning: unknown type");
  //       // g_debug ("%-23.23s Unknown type %ld \"%s\"", "", pspec->value_type,
  //       //  g_type_name (pspec->value_type));
  //     }
  //     break;
  }
  g_value_reset(&value);

  // FIXME register to prop and notify notification
  return res;
}

}  // namespace GPropToProp
}  // namespace switcher
