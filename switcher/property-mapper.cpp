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

#include "property-mapper.h"
#include "gst-utils.h"
#include <math.h>

//for python
// #ifdef HAVE_CONFIG_H
// #include "config.h"
// #ifdef HAVE_PYTHON
// #include <Python.h> 
// #endif
// #endif

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PropertyMapper,
				       "Switcher Property Mapper",
				       "mapper", 
				       "map two properties, one being slave of the other",
				       "LGPL",
				       "property-mapper",
				       "Nicolas Bouillot");

  PropertyMapper::PropertyMapper () :
    source_quiddity_ (),
    source_property_name_ (),
    sink_quiddity_ (),
    sink_quiddity_pspec_ (nullptr),
    sink_property_name_ (),
    custom_props_ (new CustomPropertyHelper ()),
    sink_min_spec_ (nullptr),
    sink_max_spec_ (nullptr),
    source_min_spec_ (nullptr),
    source_max_spec_ (nullptr),
    sink_min_ (0),
    sink_max_ (0),
    source_min_ (0),
    source_max_ (0)
  {}

  bool
  PropertyMapper::init()
  {
    install_method ("Set Source Property", //long name
		    "set-source-property", //name
		    "set the master property", //description
		    "success of fail", //return description
		    Method::make_arg_description ("Quiddity Name", //first arg long name
						  "quiddity_name", //fisrt arg name
						  "Name of the quiddity", //first arg description
						  "property Name", //first arg long name
						  "property_name", //fisrt arg name
						  "Name of the property", //first arg description
						  nullptr),
  		    (Method::method_ptr) &set_source_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, nullptr),
		    this);

    install_method ("Set Sink Property", //long name
		    "set-sink-property", //name
		    "set the slave property", //description
		    "success of fail", //return description
		    Method::make_arg_description ("Quiddity Name", //first arg long name
						  "quiddity_name", //fisrt arg name
						  "Name of the quiddity", //first arg description
						  "property Name", //first arg long name
						  "property_name", //fisrt arg name
						  "Name of the property", //first arg description
						  nullptr),
  		    (Method::method_ptr) &set_sink_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, nullptr),
		    this);


#ifdef HAVE_PYTHON
     // Py_Initialize();
     // // PyRun_SimpleString("from time import time,ctime\n"
     // // 		       "print 'Today is',ctime(time())\n");
     // PyObject *numpy_name = PyString_FromString("numpy");
     // PyObject *numpy = PyImport_Import(numpy_name);
     // Py_DECREF(numpy_name);
     // if (numpy != nullptr) 
     //   g_print ("got numpy !\n");
     // else
     //   g_print ("no numpy\n");
     
     // PyObject *numpey_name = PyString_FromString("numpey");
     // PyObject *numpey = PyImport_Import(numpey_name);
     // Py_DECREF(numpey_name);
     // if (numpey != nullptr) 
     //   g_print ("got numpey !\n");
     // else
     //   g_print ("no numpey..\n");
       
     //   Py_Finalize();
#endif
   
    return true;
  }

  PropertyMapper::~PropertyMapper ()
  {
    unsubscribe_source_property ();
  }


  void 
  PropertyMapper::unsubscribe_source_property ()
  {
    Quiddity::ptr source_quid = source_quiddity_.lock ();
    if ((bool) source_quid)
      source_quid->unsubscribe_property (source_property_name_, property_cb, this);
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

    //unsubscribing previously registered property
    context->unsubscribe_source_property ();
    context->source_quiddity_ = quid;
    context->source_property_name_ = property_name;

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
    uninstall_property ("source-min");
    source_min_spec_ =
      custom_props_->make_double_property ("source-min", 
     					   "the minimum value considered",
     					   source_min_,
     					   source_max_,
     					   source_min_,
     					   (GParamFlags) G_PARAM_READWRITE,
     					   set_double_value,
     					   get_double_value,
     					   &source_min_);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
     				source_min_spec_, 
     				"source-min",
     				"Source Property Minimum");
    
    uninstall_property ("source-max");
    source_max_spec_ =
      custom_props_->make_double_property ("source-max", 
     					   "the maximum value considered",
     					   source_min_,
     					   source_max_,
     					   source_max_,
     					   (GParamFlags) G_PARAM_READWRITE,
     					   set_double_value,
     					   get_double_value,
     					   &source_max_);
    install_property_by_pspec (custom_props_->get_gobject (), 
     				source_max_spec_, 
     				"source-max",
     				"Source Property Maximum");
  }

  void
  PropertyMapper::make_numerical_sink_properties ()
  {
    uninstall_property ("sink-min");
    sink_min_spec_ =
      custom_props_->make_double_property ("sink-min", 
     					   "the minimum value considered",
     					   sink_min_,
     					   sink_max_,
     					   sink_min_,
     					   (GParamFlags) G_PARAM_READWRITE,
     					   set_double_value,
     					   get_double_value,
     					   &sink_min_);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
     				sink_min_spec_, 
     				"sink-min",
     				"Sink Property Minimum");
    
    uninstall_property ("sink-max");
    sink_max_spec_ =
      custom_props_->make_double_property ("sink-max", 
     					   "the maximum value considered",
     					   sink_min_,
     					   sink_max_,
     					   sink_max_,
     					   (GParamFlags) G_PARAM_READWRITE,
     					   set_double_value,
     					   get_double_value,
     					   &sink_max_);
    install_property_by_pspec (custom_props_->get_gobject (), 
     				sink_max_spec_, 
     				"sink-max",
     				"Sink Property Maximum");
  }

  void 
  PropertyMapper::property_cb (GObject *gobject, GParamSpec *pspec, gpointer user_data)
  {
    PropertyMapper *context = static_cast <PropertyMapper *>(user_data);
    
    //return if not property to update
    Quiddity::ptr quid = context->sink_quiddity_.lock ();
    if (! (bool) quid)
      return;

    GValue val = G_VALUE_INIT; 
    const gchar *prop_name = g_param_spec_get_name (pspec);
    gdouble new_value = 0; 
    g_value_init (&val, pspec->value_type);
    g_object_get_property (gobject,
			   prop_name,
			   &val);
    
    //getting new value
    switch (pspec->value_type) {
    case G_TYPE_INT:
      {
	gint orig_val = g_value_get_int (&val);
	//ignoring value out of range
	if (orig_val < context->source_min_ || orig_val > context->source_max_)
	  {
	    g_value_unset (&val);
	    return;
	  }
	new_value = (gdouble) orig_val;
      }
      break;
    case G_TYPE_DOUBLE:
      {
	gdouble orig_val = g_value_get_double (&val);
	//ignoring value out of range
	if (orig_val < context->source_min_ || orig_val > context->source_max_)
	  {
	    g_value_unset (&val);
	    return;
	  }
	new_value = (gdouble) orig_val;
      }
      break;
    default:
      {
	g_debug ("property mapper callback: type %s not handled (yet)\n",
		 g_type_name (pspec->value_type));
	return;
      }
    }
    g_value_unset (&val);

    //scaling and transforming the value into a gdouble value 
    gdouble transformed_val =  
      ((gdouble)new_value - context->source_min_) * (context->sink_max_ - context->sink_min_) 
      / (context->source_max_ - context->source_min_)
      + context->sink_min_;
    
    GValue val_gdouble = G_VALUE_INIT; 
    g_value_init (&val_gdouble, G_TYPE_DOUBLE);
    g_value_set_double (&val_gdouble, transformed_val);
      
   //applying to sink property
    GValue val_to_apply = G_VALUE_INIT; 
    g_value_init (&val_to_apply, context->sink_quiddity_pspec_->value_type);

    g_value_transform (&val_gdouble, &val_to_apply);

    // g_print ("%f, %s\n", g_value_get_double(&val_gdouble), 
    // 	     GstUtils::gvalue_serialize (&val_to_apply));
    if ((bool) quid && quid->has_property (context->sink_property_name_))
      quid->set_property (context->sink_property_name_, GstUtils::gvalue_serialize (&val_to_apply)); 

    g_value_unset (&val_gdouble);
    g_value_unset (&val_to_apply);
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
    
    GParamSpec *pspec = quid->get_property_ptr (property_name)->get_paramspec ();
    switch (pspec->value_type) {
    case G_TYPE_INT:
      {
	GParamSpecInt *pint = G_PARAM_SPEC_INT (pspec);
	context->sink_min_ = pint->minimum;
	context->sink_max_ = pint->maximum;
	context->make_numerical_sink_properties ();
      }
      break;
    case G_TYPE_DOUBLE:
      {
	GParamSpecDouble *pdouble = G_PARAM_SPEC_DOUBLE (pspec);
	context->sink_min_ = pdouble->minimum;
	context->sink_max_ = pdouble->maximum;
	context->make_numerical_sink_properties ();
      }
      break;
    default:
      {
	g_debug ("type not handled (%s)", g_type_name (pspec->value_type));
	return FALSE;
      }
    }

    context->sink_quiddity_ = quid;
    context->sink_property_name_ = property_name;
    context->sink_quiddity_pspec_ = quid->get_property_ptr (property_name)->get_paramspec ();
    return TRUE;
  }

  void 
  PropertyMapper::set_double_value (gdouble value, void *user_data)
  {
    *(gdouble *)user_data = value;
  }
  
  gdouble 
  PropertyMapper::get_double_value (void *user_data)
  {
    return *(gdouble *)user_data;
  }

}
