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

#include "./gprop-to-prop.hpp"
#include "./scope-exit.hpp"

namespace switcher {
namespace GPropToProp {

std::unique_ptr<PropertyBase> to_prop(GObject* gobj, const std::string& gprop_name) {
  GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(gobj), gprop_name.c_str());
  if (nullptr == pspec) {
    return std::unique_ptr<PropertyBase>();
  }

  GValue value = G_VALUE_INIT;
  g_value_init(&value, pspec->value_type);
  g_object_get_property(gobj, pspec->name, &value);
  std::string description = std::string(g_param_spec_get_blurb(pspec));
  bool is_writable = false;
  if (pspec->flags & G_PARAM_WRITABLE) is_writable = true;

  auto res = std::unique_ptr<PropertyBase>();
  switch (G_VALUE_TYPE(&value)) {
    case G_TYPE_STRING: {
      auto content = g_value_get_string(&value);
      res = std::make_unique<Property<std::string>>(
          is_writable ?
          [gobj, gprop_name](const std::string &val){
            g_object_set(gobj, gprop_name.c_str(), val.c_str(), nullptr);
            return true;
          } : static_cast<prop::set_t<std::string>>(nullptr),  // FIXME
          [gobj, gprop_name](){
            gchar* strval = nullptr;
            On_scope_exit {
              if (strval) g_free(strval);
            };
            g_object_get(gobj, gprop_name.c_str(), &strval, nullptr);
            return std::string(strval ? strval : "");},
          gprop_name,
          std::string(description),
          std::string(content ? content : ""));
      break;
    }
    case G_TYPE_BOOLEAN:
      res = std::make_unique<Property<bool>>(
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
    case G_TYPE_ULONG: {
      GParamSpecULong* pulong = G_PARAM_SPEC_ULONG(pspec);
      res = std::make_unique<Property<gulong>>(
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
    } break;
    case G_TYPE_LONG: {
      GParamSpecLong* plong = G_PARAM_SPEC_LONG(pspec);
      res = std::make_unique<Property<glong>>(
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
    } break;
    case G_TYPE_UINT: {
      GParamSpecUInt* puint = G_PARAM_SPEC_UINT(pspec);
      res = std::make_unique<Property<guint>>(
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
    } break;
    case G_TYPE_INT: {
      GParamSpecInt* pint = G_PARAM_SPEC_INT(pspec);
      res = std::make_unique<Property<gint>>(
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
    } break;
    case G_TYPE_UINT64: {
      GParamSpecUInt64* puint64 = G_PARAM_SPEC_UINT64(pspec);
      res = std::make_unique<Property<guint64>>(
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
    } break;
    case G_TYPE_INT64: {
      GParamSpecInt64* pint64 = G_PARAM_SPEC_INT64(pspec);
      res = std::make_unique<Property<gint64>>(
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
    } break;
    case G_TYPE_FLOAT: {
      GParamSpecFloat* pfloat = G_PARAM_SPEC_FLOAT(pspec);
      res = std::make_unique<Property<gfloat>>(
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
    } break;
    case G_TYPE_DOUBLE: {
      GParamSpecDouble* pdouble = G_PARAM_SPEC_DOUBLE(pspec);
      res = std::make_unique<Property<gdouble>>(
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
    } break;
    default:
      if (pspec->value_type == GST_TYPE_CAPS) {
#ifdef DEBUG
        std::cerr << "GST_TYPE_CAPS is not a supporteed property type" << '\n';
#endif
      } else if (G_IS_PARAM_SPEC_ENUM(pspec)) {
        std::vector<std::string> items;
        GEnumValue* values = G_ENUM_CLASS(g_type_class_ref(pspec->value_type))->values;
        guint j = 0;
        // GEnumValue are *not* always from 0 to n, incremented by 1 between
        // exemple: gst-inspect opusenc
        // so here finding the index value for selecting the same default value with Selection
        gint default_value_gst = g_value_get_enum(&value);
        gint default_value = 0;
        std::map<gint /*gst values*/, gint /*index*/> indexes;
        while (values[j].value_name) {
          // values[j].value_nick;
          items.push_back(values[j].value_name);
          if (default_value_gst == values[j].value) default_value = j;
          // saving value for the getter
          indexes[values[j].value] = j;
          j++;
        }
        res = std::make_unique<Property<Selection<>, IndexOrName>>(
            is_writable ?
            [gobj, gprop_name](const IndexOrName &val){
              if(val.is_index_) {
                g_object_set(gobj, gprop_name.c_str(), val.index_, nullptr);
              } else {
                std::cout << "not is_index_ " << val.name_ << '\n';
               gst_util_set_object_arg(gobj, gprop_name.c_str(), val.name_.c_str());
                //g_object_set(gobj, gprop_name.c_str(), val.name_.c_str(), nullptr);
              }
              return true;
            } : static_cast<prop::set_t<IndexOrName>>(nullptr),
            [gobj, gprop_name, indexes](){
              gint val;
              g_object_get(gobj, gprop_name.c_str(), &val, nullptr);
              return IndexOrName(indexes.at(val));
            },
            gprop_name,
            std::string(description),
            Selection<>(std::move(items), default_value),
            items.size() - 1);
      } else if (G_IS_PARAM_SPEC_FLAGS(pspec)) {
#ifdef DEBUG
        std::cerr << "warning: param spec flags not handled" << '\n';
#endif
      } else if (G_IS_PARAM_SPEC_OBJECT(pspec)) {
#ifdef DEBUG
        std::cerr << "warning: param spec object not handled" << '\n';
#endif
      } else if (G_IS_PARAM_SPEC_BOXED(pspec)) {
#ifdef DEBUG
        std::cerr << "warning: param spec boxed not handled" << '\n';
#endif
      } else if (G_IS_PARAM_SPEC_POINTER(pspec)) {
#ifdef DEBUG
        std::cerr << "warning: param spec pointer not handled" << '\n';
#endif
      } else if (GST_IS_PARAM_SPEC_FRACTION(pspec)) {
        GstParamSpecFraction* pfraction = GST_PARAM_SPEC_FRACTION(pspec);
        res = std::make_unique<Property<Fraction>>(
            is_writable ?
            [gobj, gprop_name](const Fraction &val){
              GValue fract = G_VALUE_INIT;  On_scope_exit{g_value_reset(&fract);};
              g_value_init (&fract, GST_TYPE_FRACTION);
              gst_value_set_fraction (&fract, val.numerator(), val.denominator());
              g_object_set_property(gobj, gprop_name.c_str(), &fract);
              return true;
            } : static_cast<prop::set_t<Fraction>>(nullptr),
            [gobj, gprop_name](){
              GValue fract = G_VALUE_INIT; On_scope_exit{g_value_reset(&fract);};
              g_object_get_property(gobj, gprop_name.c_str(), &fract);
              // if (!GST_VALUE_HOLDS_FRACTION(&fract))
              //   return Fraction(0, 0);
              return Fraction(gst_value_get_fraction_numerator (&fract),
                              gst_value_get_fraction_denominator (&fract));
            },
            gprop_name,
            std::string(description),
            Fraction(gst_value_get_fraction_numerator(&value),
                     gst_value_get_fraction_denominator (&value)),
            pfraction->min_num,
            pfraction->min_den,
            pfraction->max_num,
            pfraction->max_den);
      } else {
#ifdef DEBUG
        std::cerr << "warning: gobject unknown type not handled" << '\n';
#endif
      }
      break;
  }
  g_value_reset(&value);

  // FIXME register to prop and notify notification
  return res;
}

}  // namespace GPropToProp
}  // namespace switcher
