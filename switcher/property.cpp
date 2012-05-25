/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * The Param class
 */

#include "switcher/property.h"
namespace switcher
{


  //should not be used because it is writing a name property
  Property::Property (GObject *object, GParamSpec *pspec) :
    property_ (pspec),
    object_ (object)
  {
  }
  

  //from gst-inspect
  void 
  Property::print()
  {
    
    guint i;
    gboolean readable;
    gboolean first_flag;
    

    GValue value = { 0, };
        
    GObject * element = (GObject *) g_param_spec_get_qdata (property_,
							    g_quark_from_static_string("origin_object"));
	
    readable = FALSE;
	
    g_value_init (&value, property_->value_type);
	
    g_print ("  %-20s: %s\n", g_param_spec_get_name (property_),
	     g_param_spec_get_blurb (property_));
	
    // first_flag = TRUE;
    // g_print ("%-23.23s flags: ", "");
    // if (property_->flags & G_PARAM_READABLE) {
    // 	g_object_get_property (G_OBJECT (element), property_->name, &value);
    // 	readable = TRUE;
    // 	g_print ("%s%s", (first_flag) ? "" : ", ", ("readable"));
    // 	first_flag = FALSE;
    // } else {
    // 	/* if we can't read the property value, assume it's set to the default
    // 	 * (which might not be entirely true for sub-classes, but that's an
    // 	 * unlikely corner-case anyway) */
    // 	g_param_value_set_default (property_, &value);
    // }
    // if (property_->flags & G_PARAM_WRITABLE) {
    // 	g_print ("%s%s", (first_flag) ? "" : ", ", ("writable"));
    // 	first_flag = FALSE;
    // }
    // if (property_->flags & GST_PARAM_CONTROLLABLE) {
    // 	g_print (", %s", ("controllable"));
    // 	first_flag = FALSE;
    // }
    // if (property_->flags & GST_PARAM_MUTABLE_PLAYING) {
    // 	g_print (", %s", ("changeable in NULL, READY, PAUSED or PLAYING state"));
    // } else if (property_->flags & GST_PARAM_MUTABLE_PAUSED) {
    // 	g_print (", %s", ("changeable only in NULL, READY or PAUSED state"));
    // } else if (property_->flags & GST_PARAM_MUTABLE_READY) {
    // 	g_print (", %s", ("changeable only in NULL or READY state"));
    // }
    // if (property_->flags & ~KNOWN_PARAM_FLAGS) {
    // 	g_print ("%s0x%0x", (first_flag) ? "" : ", ",
    // 		 property_->flags & ~KNOWN_PARAM_FLAGS);
    // }
    // g_print ("\n");

    switch (G_VALUE_TYPE (&value)) {
    case G_TYPE_STRING:
      {
	const char *string_val = g_value_get_string (&value);

	g_print ("%-23.23s String. ", "");

	if (string_val == NULL)
	  g_print ("Default: null");
	else
	  g_print ("Default: \"%s\"", string_val);
	break;
      }
    case G_TYPE_BOOLEAN:
      {
	gboolean bool_val = g_value_get_boolean (&value);

	g_print ("%-23.23s Boolean. ", "");

	g_print ("Default: %s", bool_val ? "true" : "false");
	break;
      }
    case G_TYPE_ULONG:
      {
	GParamSpecULong *pulong = G_PARAM_SPEC_ULONG (property_);

	g_print ("%-23.23s Unsigned Long. ", "");
	g_print ("Range: %lu - %lu Default: %lu ",
		 pulong->minimum, pulong->maximum, g_value_get_ulong (&value));
	break;
      }
    case G_TYPE_LONG:
      {
	GParamSpecLong *plong = G_PARAM_SPEC_LONG (property_);

	g_print ("%-23.23s Long. ", "");
	g_print ("Range: %ld - %ld Default: %ld ",
		 plong->minimum, plong->maximum, g_value_get_long (&value));
	break;
      }
    case G_TYPE_UINT:
      {
	GParamSpecUInt *puint = G_PARAM_SPEC_UINT (property_);

	g_print ("%-23.23s Unsigned Integer. ", "");
	g_print ("Range: %u - %u Default: %u ",
		 puint->minimum, puint->maximum, g_value_get_uint (&value));
	break;
      }
    case G_TYPE_INT:
      {
	GParamSpecInt *pint = G_PARAM_SPEC_INT (property_);

	g_print ("%-23.23s Integer. ", "");
	g_print ("Range: %d - %d Default: %d ",
		 pint->minimum, pint->maximum, g_value_get_int (&value));
	break;
      }
    case G_TYPE_UINT64:
      {
	GParamSpecUInt64 *puint64 = G_PARAM_SPEC_UINT64 (property_);

	g_print ("%-23.23s Unsigned Integer64. ", "");
	g_print ("Range: %" G_GUINT64_FORMAT " - %" G_GUINT64_FORMAT
		 " Default: %" G_GUINT64_FORMAT " ",
		 puint64->minimum, puint64->maximum, g_value_get_uint64 (&value));
	break;
      }
    case G_TYPE_INT64:
      {
	GParamSpecInt64 *pint64 = G_PARAM_SPEC_INT64 (property_);

	g_print ("%-23.23s Integer64. ", "");
	g_print ("Range: %" G_GINT64_FORMAT " - %" G_GINT64_FORMAT
		 " Default: %" G_GINT64_FORMAT " ",
		 pint64->minimum, pint64->maximum, g_value_get_int64 (&value));
	break;
      }
    case G_TYPE_FLOAT:
      {
	GParamSpecFloat *pfloat = G_PARAM_SPEC_FLOAT (property_);

	g_print ("%-23.23s Float. ", "");
	g_print ("Range: %15.7g - %15.7g Default: %15.7g ",
		 pfloat->minimum, pfloat->maximum, g_value_get_float (&value));
	break;
      }
    case G_TYPE_DOUBLE:
      {
	GParamSpecDouble *pdouble = G_PARAM_SPEC_DOUBLE (property_);

	g_print ("%-23.23s Double. ", "");
	g_print ("Range: %15.7g - %15.7g Default: %15.7g ",
		 pdouble->minimum, pdouble->maximum, g_value_get_double (&value));
	break;
      }
    default:
      if (property_->value_type == GST_TYPE_CAPS) {
	const GstCaps *caps = gst_value_get_caps (&value);

	if (!caps)
	  g_print ("%-23.23s Caps (NULL)", "");
	else {
	  g_print ("%-23.23s Caps (%s)", "", gst_caps_to_string (caps));
	}
      } else if (G_IS_PARAM_SPEC_ENUM (property_)) {
	GEnumValue *values;
	guint j = 0;
	gint enum_value;
	const gchar *value_nick = "";

	values = G_ENUM_CLASS (g_type_class_ref (property_->value_type))->values;
	enum_value = g_value_get_enum (&value);

	while (values[j].value_name) {
	  if (values[j].value == enum_value)
	    value_nick = values[j].value_nick;
	  j++;
	}

	g_print ("%-23.23s Enum \"%s\" Default: %d, \"%s\"", "",
		 g_type_name (G_VALUE_TYPE (&value)), enum_value, value_nick);

	j = 0;
	while (values[j].value_name) {
	  g_print ("\n");
	  // if (_name)
	  //   g_print ("%s", _name);
	  g_print ("%-23.23s    (%d): %-16s - %s", "",
		   values[j].value, values[j].value_nick, values[j].value_name);
	  j++;
	}
	/* g_type_class_unref (ec); */
      } else if (G_IS_PARAM_SPEC_FLAGS (property_)) {
	// GParamSpecFlags *pflags = G_PARAM_SPEC_FLAGS (property_);
	// GFlagsValue *vals;
	// gchar *cur;

	// vals = pflags->flags_class->values;

	// cur = flags_to_string (vals, g_value_get_flags (&value));

	// g_print ("%-23.23s Flags \"%s\" Default: 0x%08x, \"%s\"", "",
	// 	   g_type_name (G_VALUE_TYPE (&value)),
	// 	   g_value_get_flags (&value), cur);

	// while (vals[0].value_name) {
	//   g_print ("\n");
	//   if (_name)
	//     g_print ("%s", _name);
	//   g_print ("%-23.23s    (0x%08x): %-16s - %s", "",
	// 	     vals[0].value, vals[0].value_nick, vals[0].value_name);
	//   ++vals;
	// }

	// g_free (cur);
      } else if (G_IS_PARAM_SPEC_OBJECT (property_)) {
	g_print ("%-23.23s Object of type \"%s\"", "",
		 g_type_name (property_->value_type));
      } else if (G_IS_PARAM_SPEC_BOXED (property_)) {
	g_print ("%-23.23s Boxed pointer of type \"%s\"", "",
		 g_type_name (property_->value_type));
      } else if (G_IS_PARAM_SPEC_POINTER (property_)) {
	if (property_->value_type != G_TYPE_POINTER) {
	  g_print ("%-23.23s Pointer of type \"%s\".", "",
		   g_type_name (property_->value_type));
	} else {
	  g_print ("%-23.23s Pointer.", "");
	}
      } else if (property_->value_type == G_TYPE_VALUE_ARRAY) {
	GParamSpecValueArray *pvarray = G_PARAM_SPEC_VALUE_ARRAY (property_);

	if (pvarray->element_spec) {
	  g_print ("%-23.23s Array of GValues of type \"%s\"", "",
		   g_type_name (pvarray->element_spec->value_type));
	} else {
	  g_print ("%-23.23s Array of GValues", "");
	}
      } else if (GST_IS_PARAM_SPEC_FRACTION (property_)) {
	GstParamSpecFraction *pfraction = GST_PARAM_SPEC_FRACTION (property_);

	g_print ("%-23.23s Fraction. ", "");

	g_print ("Range: %d/%d - %d/%d Default: %d/%d ",
		 pfraction->min_num, pfraction->min_den,
		 pfraction->max_num, pfraction->max_den,
		 gst_value_get_fraction_numerator (&value),
		 gst_value_get_fraction_denominator (&value));
      } else if (GST_IS_PARAM_SPEC_MINI_OBJECT (property_)) {
	g_print ("%-23.23s MiniObject of type \"%s\"", "",
		 g_type_name (property_->value_type));
      } else {
	g_print ("%-23.23s Unknown type %ld \"%s\"", "", property_->value_type,
		 g_type_name (property_->value_type));
      }
      break;
    }
    if (!readable)
      g_print (" Write only\n");
    else
      g_print ("\n");

    g_value_reset (&value);
  }
 
 
}
