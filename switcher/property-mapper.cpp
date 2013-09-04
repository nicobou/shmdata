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

#include "property-mapper.h"
#include "gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PropertyMapper,
				       "Switcher Property Mapper",
				       "mapper", 
				       "map two properties, one being slave of the other",
				       "LGPL",
				       "property-mapper",
				       "Nicolas Bouillot");
  bool
  PropertyMapper::init()
  {
    publish_method ("Set Source Property", //long name
		    "set-source-property", //name
		    "set the master property", //description
		    "success of fail", //return description
		    Method::make_arg_description ("Quiddity Name", //first arg long name
						  "quiddity_name", //fisrt arg name
						  "Name of the quiddity", //first arg description
						  "property Name", //first arg long name
						  "property_name", //fisrt arg name
						  "Name of the property", //first arg description
						  NULL),
  		    (Method::method_ptr) &set_source_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
		    this);

    publish_method ("Set Sink Property", //long name
		    "set-sink-property", //name
		    "set the slave property", //description
		    "success of fail", //return description
		    Method::make_arg_description ("Quiddity Name", //first arg long name
						  "quiddity_name", //fisrt arg name
						  "Name of the quiddity", //first arg description
						  "property Name", //first arg long name
						  "property_name", //fisrt arg name
						  "Name of the property", //first arg description
						  NULL),
  		    (Method::method_ptr) &set_sink_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
		    this);

    
    
    return true;
  }

  PropertyMapper::~PropertyMapper ()
  {
    
  }

  gboolean
  PropertyMapper::set_source_property_method (gchar *quiddity_name, 
					      gchar *property_name, 
					      void *user_data)
  {
    PropertyMapper *context = static_cast<PropertyMapper *> (user_data);
    
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    
    if (!(bool) manager)
      {
	g_debug ("manager not found");
	return FALSE;
      }
    
    Quiddity::ptr quid = manager->get_quiddity (quiddity_name);
    
    if (!(bool) quid)
      {
	g_debug ("quiddity %s not found",
		 quiddity_name);
	return FALSE;
      }
    
    if (!quid->subscribe_property(property_name, 
				  property_cb, 
				  context))
      return FALSE;

    GParamSpec *pspec = quid->get_property_ptr (property_name)->get_paramspec ();
    switch (pspec->value_type) {
    case G_TYPE_INT:
      {
	GParamSpecInt *pint = G_PARAM_SPEC_INT (pspec);
	context->source_min_ = pint->minimum;
	context->source_max_ = pint->maximum;
	context->make_numerical_source_properties ();
      }
      break;
    case G_TYPE_DOUBLE:
      {
	GParamSpecDouble *pdouble = G_PARAM_SPEC_DOUBLE (pspec);
	context->source_min_ = pdouble->minimum;
	context->source_max_ = pdouble->maximum;
	context->make_numerical_source_properties ();
      }
      break;
    default:
      g_debug ("type not handled (%s)", g_type_name (pspec->value_type));
    }
    return TRUE;
  }

  void
  PropertyMapper::make_numerical_source_properties ()
  {
    // unregister_property ("source-min");
    // source_min_spec_ =
    //   custom_props_->make_double_property ("source-min", 
    // 					   "the minimum value considered",
    // 					   source_min_,
    // 					   source_max,
    // 					   source_min_,
    // 					   (GParamFlags) G_PARAM_READWRITE,
    // 					   set_double_value,
    // 					   get_double_value,
    // 					   &source_min_);
    
    // register_property_by_pspec (custom_props_->get_gobject (), 
    // 				source_min_spec_, 
    // 				"source-min",
    // 				"Source Property Minimum");

    // unregister_property ("source-max");
    // source_max_spec_ =
    //   custom_props_->make_double_property ("source-max", 
    // 					   "the maximum value considered",
    // 					   source_min_,
    // 					   source_max,
    // 					   source_max_,
    // 					   (GParamFlags) G_PARAM_READWRITE,
    // 					   set_double_value,
    // 					   get_double_value,
    // 					   &source_min_);
    // register_property_by_pspec (custom_props_->get_gobject (), 
    // 				source_max_spec_, 
    // 				"source-max",
    // 				"Source Property Maximum");
  }

  void 
  PropertyMapper::property_cb (GObject *gobject, GParamSpec *pspec, gpointer user_data)
  {
    PropertyMapper *context = static_cast <PropertyMapper *>(user_data);
    
    GValue val = G_VALUE_INIT;
    const gchar *prop_name = g_param_spec_get_name (pspec);
    Quiddity::ptr quid = context->sink_quiddity_.lock ();
    
    switch (pspec->value_type) {
    case G_TYPE_INT:
      {
	g_value_init (&val, pspec->value_type);
	g_object_get_property (gobject,
			       prop_name,
			       &val);
	
	gint orig_val = g_value_get_int (&val);
	g_value_set_int (&val, orig_val);
      }
      break;
    default:
      g_debug ("property mapper callback: type %s unknown\n",
	       g_type_name (pspec->value_type));
    }

    if ((bool) quid 
	&& quid->has_property (context->sink_property_name_))
	quid->set_property (context->sink_property_name_, GstUtils::gvalue_serialize (&val)); 

  }
  
  gboolean
  PropertyMapper::set_sink_property_method (gchar *quiddity_name, 
					    gchar *property_name, 
					    void *user_data)
  {
    PropertyMapper *context = static_cast<PropertyMapper *> (user_data);
    
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    
    if (!(bool) manager)
      {
	g_debug ("manager not found");
	return FALSE;
      }
    
    Quiddity::ptr quid  = manager->get_quiddity (quiddity_name);
    
    if (!(bool) quid)
      {
	g_debug ("quiddity %s not found",
		 quiddity_name);
	return FALSE;
      }
    
    if (!quid->has_property (property_name))
      {
	g_debug ("quiddity %s does not have a property named %s",
		 quiddity_name,
		 property_name);
	return FALSE;
      }
    
    context->sink_quiddity_ = quid;
    context->sink_property_name_ = property_name;

    return TRUE;
  }

  void 
  PropertyMapper::set_double_value (gdouble *value, void *user_data)
  {
     *(gdouble *)user_data = *value;
  }
  
  gdouble 
  PropertyMapper::get_double_value (void *user_data)
  {
    return *(gdouble *)user_data;
  }
}
