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

/**
 * The Param class
 */

#include "property.h"
#include "gst-utils.h"

namespace switcher
{

  Property::Property () :
    long_name_ ("undefined_long_name"),
    name_ ("undefined_name"),
    property_ (NULL),
    object_ (NULL)
  {
    json_description_.reset (new JSONBuilder());
  }

  Property::~Property ()
  {
    g_param_spec_ref (property_);
    g_object_unref (object_);
  }
  
  Property::Property (const Property &source)
  {
    copy_property (source);
  }

  Property& 
  Property::operator= (const Property &source)
  {
    copy_property (source);
    return *this;
  }
  
  void
  Property::copy_property(const Property &source)
  {
    long_name_ = source.long_name_;
    name_ = source.name_;
    json_description_ = source.json_description_;
    subscribed_handlers_ = source.subscribed_handlers_;
    property_ = NULL;
    object_ = NULL;
    if (source.property_ != NULL)
      {
	g_param_spec_ref (source.property_);
	property_ = source.property_;
      }
    if (source.object_ != NULL)
      {
	g_object_ref (source.object_);   
	object_ = source.object_;
      }
  }

  void
  Property::set_gobject_pspec (GObject *object, 
			       GParamSpec *pspec)
  {
    g_param_spec_ref (pspec);
    g_object_ref (object);
    property_ = pspec;
    object_ = object;
  }
  
  void
  Property::set_long_name (std::string long_name)
  {
    long_name_ = long_name;
  }

  void
  Property::set_name (std::string name)
  {
    name_ = name;
  }
  
  std::string
  Property::get_name ()
  {
    return name_;
  }

  void 
  Property::set (std::string value)
  {
    
    GValue transformed_val = G_VALUE_INIT;
    g_value_init (&transformed_val, property_->value_type);

    if (!gst_value_deserialize (&transformed_val,value.c_str()))
      g_debug ("string not transformable into gvalue ");
    
    g_object_set_property (object_,
			   property_->name, 
			   &transformed_val);
  }

  bool
  Property::subscribe (Callback cb, void *user_data)
  {
    std::pair <Callback, void *> subscribe_id = std::make_pair (cb, user_data);
    gchar *signal = g_strconcat ("notify::", property_->name, NULL);
    if (subscribed_handlers_.find(subscribe_id) != subscribed_handlers_.end ())
      {
	g_debug ("cannot subscribe callback/user_data");
	return false;
      }
    
    subscribed_handlers_[subscribe_id] = g_signal_connect (object_, signal, G_CALLBACK (cb), user_data);
    return true;

  }

  bool
  Property::unsubscribe (Callback cb, void *user_data)
  {
    std::pair <Callback, void *> subscribe_id = std::make_pair (cb, user_data);
    if (subscribed_handlers_.find(subscribe_id) == subscribed_handlers_.end ())
      return false;
    
    g_signal_handler_disconnect (object_, subscribed_handlers_[subscribe_id]);
    subscribed_handlers_.erase (subscribe_id);
    return true;
  }

  std::string
  Property::parse_callback_args (GObject *gobject, GParamSpec *pspec)
  {
    const gchar *prop_name = g_param_spec_get_name (pspec);
    GValue val = G_VALUE_INIT;
    g_value_init (&val, pspec->value_type);
    g_object_get_property (gobject,
			   prop_name,
			   &val);
    gchar *val_str = GstUtils::gvalue_serialize (&val);
    //gchar *val_str;
    // if (pspec->value_type == G_TYPE_STRING)
    //   val_str = g_strdup (g_value_get_string (&val));
    // else
    //   val_str = gst_value_serialize (&val);
    std::string res (val_str);
    g_free (val_str);
    g_value_unset (&val);
    return res;
  }
  
  std::string 
  Property::get ()
  {
    GValue val = G_VALUE_INIT;
    g_value_init (&val, property_->value_type);
    
    g_object_get_property (object_,
			   property_->name,
			   &val);
    
    gchar *val_str = GstUtils::gvalue_serialize (&val);
    std::string res (val_str);
    g_free (val_str);
    return res;
  }
 
  std::string
  Property::get_description ()
  {
    make_description ();
    return json_description_->get_string(true);
  }

  JSONBuilder::Node
  Property::get_json_root_node ()
  {
    make_description ();
    return json_description_->get_root ();
  }

  //make json formated description 
  void
  Property::make_description ()
  {
    // guint i;
    //gboolean readable = FALSE;
    // gboolean first_flag;
    GValue value = G_VALUE_INIT;
    // GObject *element = object_; 
    g_value_init (&value, property_->value_type);
    
    g_object_get_property (object_,
			   property_->name,
			   &value);

    json_description_->reset ();
    json_description_->begin_object ();

    //long name 
    json_description_->add_string_member ("long name", long_name_.c_str ());

    //name
    json_description_->add_string_member ("name", name_.c_str ());

    //nickname 
    //json_description_->add_string_member ("nickname", g_param_spec_get_nick (property_));

    // short description
    json_description_->add_string_member ("short description", g_param_spec_get_blurb (property_));
    
    json_description_->add_string_member ("position category", get_category ().c_str ());
    json_description_->add_int_member    ("position weight", get_position_weight ());

    // name
    //json_description_->add_string_member ("internal name", g_param_spec_get_name (property_));

    if (property_->flags & G_PARAM_WRITABLE) 
      json_description_->add_string_member ("writable", "true");
    else
      json_description_->add_string_member ("writable", "false");
    
    switch (G_VALUE_TYPE (&value)) {
    case G_TYPE_STRING:
      {
	const char *string_val = g_value_get_string (&value);
	json_description_->add_string_member ("type", "string");
	if (string_val == NULL)
	  json_description_->add_string_member ("default value","");
	else
	  json_description_->add_string_member ("default value",string_val);
	break;
      }
    case G_TYPE_BOOLEAN:
      {
	gboolean bool_val = g_value_get_boolean (&value);
	json_description_->add_string_member ("type","boolean");
	if (bool_val)
	  json_description_->add_string_member ("default value","true");
	else
	  json_description_->add_string_member ("default value","false");
	break;
      }
    case G_TYPE_ULONG:
      {
	GParamSpecULong *pulong = G_PARAM_SPEC_ULONG (property_);

	json_description_->add_string_member ("type","ulong");
	gchar *min = g_strdup_printf ("%lu",pulong->minimum);
	gchar *max = g_strdup_printf ("%lu",pulong->maximum);
	gchar *default_value = g_strdup_printf ("%lu",g_value_get_ulong (&value));
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
	break;
      }
    case G_TYPE_LONG:
      {
	GParamSpecLong *plong = G_PARAM_SPEC_LONG (property_);
	gchar *min = g_strdup_printf ("%ld",plong->minimum);
	gchar *max = g_strdup_printf ("%ld",plong->maximum);
	gchar *default_value = g_strdup_printf ("%ld",g_value_get_ulong (&value));
	json_description_->add_string_member ("type","long");
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
	break;
      }
    case G_TYPE_UINT:
      {
	GParamSpecUInt *puint = G_PARAM_SPEC_UINT (property_);
	gchar *min = g_strdup_printf ("%u",puint->minimum);
	gchar *max = g_strdup_printf ("%u",puint->maximum);
	gchar *default_value = g_strdup_printf ("%u",g_value_get_uint (&value));
	json_description_->add_string_member ("type","uint");
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
	break;
      }
    case G_TYPE_INT:
      {
	GParamSpecInt *pint = G_PARAM_SPEC_INT (property_);
	gchar *min = g_strdup_printf ("%d",pint->minimum);
	gchar *max = g_strdup_printf ("%d",pint->maximum);
	gchar *default_value = g_strdup_printf ("%d",g_value_get_int (&value));
	json_description_->add_string_member ("type","int");
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
	break;
      }
    case G_TYPE_UINT64:
      {
	GParamSpecUInt64 *puint64 = G_PARAM_SPEC_UINT64 (property_);
	gchar *min = g_strdup_printf ("%" G_GUINT64_FORMAT,puint64->minimum);
	gchar *max = g_strdup_printf ("%" G_GUINT64_FORMAT,puint64->maximum);
	gchar *default_value = g_strdup_printf ("%" G_GUINT64_FORMAT,g_value_get_uint64 (&value));
	json_description_->add_string_member ("type","uint64");
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
	break;
      }
    case G_TYPE_INT64:
      {
     	GParamSpecInt64 *pint64 = G_PARAM_SPEC_INT64 (property_);
	gchar *min = g_strdup_printf ("%" G_GINT64_FORMAT,pint64->minimum);
	gchar *max = g_strdup_printf ("%" G_GINT64_FORMAT,pint64->maximum);
	gchar *default_value = g_strdup_printf ("%" G_GINT64_FORMAT,g_value_get_int64 (&value));
	json_description_->add_string_member ("type","int64");
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
     	break;
      }
    case G_TYPE_FLOAT:
      {
     	GParamSpecFloat *pfloat = G_PARAM_SPEC_FLOAT (property_);
	gchar *min = g_strdup_printf ("%.7g",pfloat->minimum);
	gchar *max = g_strdup_printf ("%.7g",pfloat->maximum);
	gchar *default_value = g_strdup_printf ("%.7g",g_value_get_float (&value));
	json_description_->add_string_member ("type","float");
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
     	break;
      }
    case G_TYPE_DOUBLE:
      {
     	GParamSpecDouble *pdouble = G_PARAM_SPEC_DOUBLE (property_);
	gchar *min = g_strdup_printf ("%.7g",pdouble->minimum);
	gchar *max = g_strdup_printf ("%.7g",pdouble->maximum);
	gchar *default_value = g_strdup_printf ("%.7g",g_value_get_double (&value));
	json_description_->add_string_member ("type","double");
	json_description_->add_string_member ("minimum",min);
	json_description_->add_string_member ("maximum",max);
	json_description_->add_string_member ("default value",default_value);
	g_free (min);
	g_free (max);
	g_free (default_value); 
     	break;
      }
    default:
      if (property_->value_type == GST_TYPE_CAPS) {
	const GstCaps *caps = gst_value_get_caps (&value);
	json_description_->add_string_member ("type","caps");
	if (!caps)
	  json_description_->add_string_member ("default value","");
	else 
	  json_description_->add_string_member ("default value",gst_caps_to_string (caps));
      } else if (G_IS_PARAM_SPEC_ENUM (property_)) {
	GEnumValue *values;
	guint j = 0;
	gint enum_value;
	const gchar *value_nick = "";
	const gchar *value_name = "";
	json_description_->add_string_member ("type","enum");
	values = G_ENUM_CLASS (g_type_class_ref (property_->value_type))->values;
	enum_value = g_value_get_enum (&value);
	while (values[j].value_name) {
	  if (values[j].value == enum_value)
	    {
	      value_nick = values[j].value_nick;
	      value_name = values[j].value_name;
	    }
	  j++;
	}

	json_description_->set_member_name ("default value");
	json_description_->begin_object ();
	gchar *value = g_strdup_printf ("%d",enum_value);
	json_description_->add_string_member ("value",value);
	g_free (value);
	json_description_->add_string_member ("nick",value_nick);
	json_description_->add_string_member ("name",value_name);
	json_description_->end_object ();

	// g_debug ("Enum \"%s\" Default: %d, \"%s\" \"%s\"",
	// 	 g_type_name (G_VALUE_TYPE (&value)), 
	// 	 enum_value, 
	// 	 value_nick,
	// 	 value_name);

	j = 0;

	json_description_->set_member_name ("values");
	json_description_->begin_array ();
	 while (values[j].value_name) {
	   json_description_->begin_object ();
	   json_description_->add_string_member ("name",values[j].value_name);
	   json_description_->add_string_member ("nick",values[j].value_nick);
	   gchar *values_value = g_strdup_printf ("%d",values[j].value);
	   json_description_->add_string_member ("value",values_value);
	   g_free (values_value);
	   json_description_->end_object ();
	   j++;
	 }
	json_description_->end_array ();
	
	/* g_type_class_unref (ec); */
      } else if (G_IS_PARAM_SPEC_FLAGS (property_)) {
	g_debug ("warning: param spec flags not handled");
	// GParamSpecFlags *pflags = G_PARAM_SPEC_FLAGS (property_);
	// GFlagsValue *vals;
	// gchar *cur;

	// vals = pflags->flags_class->values;

	// cur = flags_to_string (vals, g_value_get_flags (&value));

	// g_debug ("%-23.23s Flags \"%s\" Default: 0x%08x, \"%s\"", "",
	// 	   g_type_name (G_VALUE_TYPE (&value)),
	// 	   g_value_get_flags (&value), cur);

	// while (vals[0].value_name) {
	//   g_debug ("");
	//   if (_name)
	//     g_debug ("%s", _name);
	//   g_debug ("%-23.23s    (0x%08x): %-16s - %s", "",
	// 	     vals[0].value, vals[0].value_nick, vals[0].value_name);
	//   ++vals;
	// }

	// g_free (cur);
      } else if (G_IS_PARAM_SPEC_OBJECT (property_)) {
	g_debug ("warning: param spec object not handled");
	// g_debug ("%-23.23s Object of type \"%s\"", "",
	// 	 g_type_name (property_->value_type));
      } else if (G_IS_PARAM_SPEC_BOXED (property_)) {
	g_debug ("warning: param spec boxed not handled");
	// g_debug ("%-23.23s Boxed pointer of type \"%s\"", "",
	// 	 g_type_name (property_->value_type));
      } else if (G_IS_PARAM_SPEC_POINTER (property_)) {
	g_debug ("warning: param spec pointer not handled");
	// if (property_->value_type != G_TYPE_POINTER) {
	//   g_debug ("%-23.23s Pointer of type \"%s\".", "",
	// 	   g_type_name (property_->value_type));
	// } else if (property_->value_type == G_TYPE_VALUE_ARRAY) {
	//GParamSpecValueArray *pvarray = G_PARAM_SPEC_VALUE_ARRAY (property_);
        // g_debug ("warning: array not handled");
	// if (pvarray->element_spec) {
	//   g_debug ("%-23.23s Array of GValues of type \"%s\"", "",
	// 	   g_type_name (pvarray->element_spec->value_type));
	// } else {
	//   g_debug ("%-23.23s Array of GValues", "");
	// }
      } else if (GST_IS_PARAM_SPEC_FRACTION (property_)) {
	GstParamSpecFraction *pfraction = GST_PARAM_SPEC_FRACTION (property_);
	json_description_->add_string_member ("type","fraction");
	gchar *minnum = g_strdup_printf ("%d",pfraction->min_num);
	gchar *minden = g_strdup_printf ("%d",pfraction->min_den);
	gchar *maxnum = g_strdup_printf ("%d",pfraction->max_num);
	gchar *maxden = g_strdup_printf ("%d",pfraction->max_den);
	gchar *defaultnum = g_strdup_printf ("%d",gst_value_get_fraction_numerator (&value));
	gchar *defaultden = g_strdup_printf ("%d",gst_value_get_fraction_denominator (&value));
	json_description_->add_string_member ("minimum numerator",minnum);
	json_description_->add_string_member ("maximum numerator",minden);
	json_description_->add_string_member ("minimum denominator",maxnum);
	json_description_->add_string_member ("maximum denominator",maxden);
	json_description_->add_string_member ("default numerator",defaultnum);
	json_description_->add_string_member ("default denominator",defaultden);
	g_free (minnum);
	g_free (minden);
	g_free (maxnum);
	g_free (maxden);
	g_free (defaultnum);
	g_free (defaultden);
	// g_debug ("Range: %d/%d - %d/%d Default: %d/%d ",
	// 	 pfraction->min_num, pfraction->min_den,
	// 	 pfraction->max_num, pfraction->max_den,
	// 	 gst_value_get_fraction_numerator (&value),
	// 	 gst_value_get_fraction_denominator (&value));
      } else if (GST_IS_PARAM_SPEC_MINI_OBJECT (property_)) {
	//g_warning ("warning param spec mini object not handled ");
	//g_warning ("%-23.23s MiniObject of type \"%s\"", "",
	//	   g_type_name (property_->value_type));
	json_description_->add_string_member ("type",g_type_name (property_->value_type));
      } else {
	g_warning ("warning: unknown type");
	// g_debug ("%-23.23s Unknown type %ld \"%s\"", "", property_->value_type,
	// 	 g_type_name (property_->value_type));
      }
      break;
    }
    g_value_reset (&value);
    json_description_->end_object ();
  }
  
  
  //from gst-inspect
  void 
  Property::print()
  {
    
    //guint i;
    gboolean readable;
    //gboolean first_flag;
    

    GValue value = G_VALUE_INIT;
        
    //GObject *element = object_; 
	
    readable = FALSE;
	
    g_value_init (&value, property_->value_type);
	
    g_debug ("  %-20s: %s", g_param_spec_get_name (property_),
	     g_param_spec_get_blurb (property_));
	
    // first_flag = TRUE;
    // g_debug ("%-23.23s flags: ", "");
    // if (property_->flags & G_PARAM_READABLE) {
    // 	g_object_get_property (G_OBJECT (element), property_->name, &value);
    // 	readable = TRUE;
    // 	g_debug ("%s%s", (first_flag) ? "" : ", ", ("readable"));
    // 	first_flag = FALSE;
    // } else {
    // 	/* if we can't read the property value, assume it's set to the default
    // 	 * (which might not be entirely true for sub-classes, but that's an
    // 	 * unlikely corner-case anyway) */
    // 	g_param_value_set_default (property_, &value);
    // }
    // if (property_->flags & G_PARAM_WRITABLE) {
    // 	g_debug ("%s%s", (first_flag) ? "" : ", ", ("writable"));
    // 	first_flag = FALSE;
    // }
    // if (property_->flags & GST_PARAM_CONTROLLABLE) {
    // 	g_debug (", %s", ("controllable"));
    // 	first_flag = FALSE;
    // }
    // if (property_->flags & GST_PARAM_MUTABLE_PLAYING) {
    // 	g_debug (", %s", ("changeable in NULL, READY, PAUSED or PLAYING state"));
    // } else if (property_->flags & GST_PARAM_MUTABLE_PAUSED) {
    // 	g_debug (", %s", ("changeable only in NULL, READY or PAUSED state"));
    // } else if (property_->flags & GST_PARAM_MUTABLE_READY) {
    // 	g_debug (", %s", ("changeable only in NULL or READY state"));
    // }
    // if (property_->flags & ~KNOWN_PARAM_FLAGS) {
    // 	g_debug ("%s0x%0x", (first_flag) ? "" : ", ",
    // 		 property_->flags & ~KNOWN_PARAM_FLAGS);
    // }
    // g_debug ("");

    switch (G_VALUE_TYPE (&value)) {
    case G_TYPE_STRING:
      {
	const char *string_val = g_value_get_string (&value);

	g_debug ("%-23.23s String. ", "");

	if (string_val == NULL)
	  g_debug ("Default: null");
	else
	  g_debug ("Default: \"%s\"", string_val);
	break;
      }
    case G_TYPE_BOOLEAN:
      {
	gboolean bool_val = g_value_get_boolean (&value);

	g_debug ("%-23.23s Boolean. ", "");

	g_debug ("Default: %s", bool_val ? "true" : "false");
	break;
      }
    case G_TYPE_ULONG:
      {
	GParamSpecULong *pulong = G_PARAM_SPEC_ULONG (property_);

	g_debug ("%-23.23s Unsigned Long. ", "");
	g_debug ("Range: %lu - %lu Default: %lu ",
		 pulong->minimum, pulong->maximum, g_value_get_ulong (&value));
	break;
      }
    case G_TYPE_LONG:
      {
	GParamSpecLong *plong = G_PARAM_SPEC_LONG (property_);

	g_debug ("%-23.23s Long. ", "");
	g_debug ("Range: %ld - %ld Default: %ld ",
		 plong->minimum, plong->maximum, g_value_get_long (&value));
	break;
      }
    case G_TYPE_UINT:
      {
	GParamSpecUInt *puint = G_PARAM_SPEC_UINT (property_);

	g_debug ("%-23.23s Unsigned Integer. ", "");
	g_debug ("Range: %u - %u Default: %u ",
		 puint->minimum, puint->maximum, g_value_get_uint (&value));
	break;
      }
    case G_TYPE_INT:
      {
	GParamSpecInt *pint = G_PARAM_SPEC_INT (property_);

	g_debug ("%-23.23s Integer. ", "");
	g_debug ("Range: %d - %d Default: %d ",
		 pint->minimum, pint->maximum, g_value_get_int (&value));
	break;
      }
    case G_TYPE_UINT64:
      {
	GParamSpecUInt64 *puint64 = G_PARAM_SPEC_UINT64 (property_);

	g_debug ("%-23.23s Unsigned Integer64. ", "");
	g_debug ("Range: %" G_GUINT64_FORMAT " - %" G_GUINT64_FORMAT
		 " Default: %" G_GUINT64_FORMAT " ",
		 puint64->minimum, puint64->maximum, g_value_get_uint64 (&value));
	break;
      }
    case G_TYPE_INT64:
      {
	GParamSpecInt64 *pint64 = G_PARAM_SPEC_INT64 (property_);

	g_debug ("%-23.23s Integer64. ", "");
	g_debug ("Range: %" G_GINT64_FORMAT " - %" G_GINT64_FORMAT
		 " Default: %" G_GINT64_FORMAT " ",
		 pint64->minimum, pint64->maximum, g_value_get_int64 (&value));
	break;
      }
    case G_TYPE_FLOAT:
      {
	GParamSpecFloat *pfloat = G_PARAM_SPEC_FLOAT (property_);

	g_debug ("%-23.23s Float. ", "");
	g_debug ("Range: %15.7g - %15.7g Default: %15.7g ",
		 pfloat->minimum, pfloat->maximum, g_value_get_float (&value));
	break;
      }
    case G_TYPE_DOUBLE:
      {
	GParamSpecDouble *pdouble = G_PARAM_SPEC_DOUBLE (property_);

	g_debug ("%-23.23s Double. ", "");
	g_debug ("Range: %15.7g - %15.7g Default: %15.7g ",
		 pdouble->minimum, pdouble->maximum, g_value_get_double (&value));
	break;
      }
    default:
      if (property_->value_type == GST_TYPE_CAPS) {
	const GstCaps *caps = gst_value_get_caps (&value);

	if (!caps)
	  g_debug ("%-23.23s Caps (NULL)", "");
	else {
	  g_debug ("%-23.23s Caps (%s)", "", gst_caps_to_string (caps));
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

	g_debug ("Enum \"%s\" Default: %d, \"%s\"", 
		 g_type_name (G_VALUE_TYPE (&value)), enum_value, value_nick);

	j = 0;
	while (values[j].value_name) {
	  // if (_name)
	  //   g_debug ("%s", _name);
	  g_debug ("%-23.23s    (%d): %-16s - %s", "",
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

	// g_debug ("%-23.23s Flags \"%s\" Default: 0x%08x, \"%s\"", "",
	// 	   g_type_name (G_VALUE_TYPE (&value)),
	// 	   g_value_get_flags (&value), cur);

	// while (vals[0].value_name) {
	//   g_debug ("");
	//   if (_name)
	//     g_debug ("%s", _name);
	//   g_debug ("%-23.23s    (0x%08x): %-16s - %s", "",
	// 	     vals[0].value, vals[0].value_nick, vals[0].value_name);
	//   ++vals;
	// }

	// g_free (cur);
      } else if (G_IS_PARAM_SPEC_OBJECT (property_)) {
	g_debug ("%-23.23s Object of type \"%s\"", "",
		 g_type_name (property_->value_type));
      } else if (G_IS_PARAM_SPEC_BOXED (property_)) {
	g_debug ("%-23.23s Boxed pointer of type \"%s\"", "",
		 g_type_name (property_->value_type));
      } else if (G_IS_PARAM_SPEC_POINTER (property_)) {
	if (property_->value_type != G_TYPE_POINTER) {
	  g_debug ("%-23.23s Pointer of type \"%s\".", "",
		   g_type_name (property_->value_type));
	} else {
	  g_debug ("%-23.23s Pointer.", "");
	}
      // } else if (property_->value_type == G_TYPE_VALUE_ARRAY) {
      // 	GParamSpecValueArray *pvarray = G_PARAM_SPEC_VALUE_ARRAY (property_);

      // 	if (pvarray->element_spec) {
      // 	  g_debug ("%-23.23s Array of GValues of type \"%s\"", "",
      // 		   g_type_name (pvarray->element_spec->value_type));
      // 	} else {
      // 	  g_debug ("%-23.23s Array of GValues", "");
      // 	}
      } else if (GST_IS_PARAM_SPEC_FRACTION (property_)) {
	GstParamSpecFraction *pfraction = GST_PARAM_SPEC_FRACTION (property_);

	g_debug ("%-23.23s Fraction. ", "");

	g_debug ("Range: %d/%d - %d/%d Default: %d/%d ",
		 pfraction->min_num, pfraction->min_den,
		 pfraction->max_num, pfraction->max_den,
		 gst_value_get_fraction_numerator (&value),
		 gst_value_get_fraction_denominator (&value));
      } else if (GST_IS_PARAM_SPEC_MINI_OBJECT (property_)) {
	g_debug ("%-23.23s MiniObject of type \"%s\"", "",
		 g_type_name (property_->value_type));
      } else {
	g_debug ("%-23.23s Unknown type %ld \"%s\"", "", property_->value_type,
		 g_type_name (property_->value_type));
      }
      break;
    }
    if (!readable)
      g_debug (" Write only");

    g_value_reset (&value);
  }

  std::string
  Property::get_short_description ()
  {
    return g_param_spec_get_blurb (property_);
  }

  std::string
  Property::get_long_name ()
  {
    return long_name_;
  }

  GObject *
  Property::get_gobject ()
  {
    return object_;
  }
  
  GParamSpec *
  Property::get_paramspec ()
  {
    return property_;
  }
}
