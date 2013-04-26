/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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
 * Class wrapping gobject for custum properties
 */

#include "gobject-wrapper.h"
#include <glib/gprintf.h>


namespace switcher
{

  //gobject
  typedef struct _MyObject {
    GObject parent_instance;
    void *context;
    // gint foo;
    // gboolean bar;
    // gchar *baz;
  } MyObject;

  typedef struct _MyObjectClass {
    GObjectClass parent_class;
  } MyObjectClass;

  static GType my_object_get_type (void);
  //clean the two following lines
  // enum { PROP_0, PROP_FOO, PROP_BAR, PROP_BAZ, N_PROPERTIES };
  // static GParamSpec *properties[N_PROPERTIES] = { NULL, };
  G_DEFINE_TYPE (MyObject, my_object, G_TYPE_OBJECT);

  // static void
  // my_object_set_foo (MyObject *obj,
  // 		     gint foo)
  // {
  //   if (obj->foo != foo)
  //     {
  // 	obj->foo = foo;
  // 	//g_object_notify_by_pspec (G_OBJECT (obj), properties[PROP_FOO]);
  // 	g_object_notify_by_pspec (G_OBJECT (obj), 
  // 				  g_object_class_find_property (G_OBJECT_GET_CLASS (obj), "coucou"));
  //     }
  // }
  
  // static void
  // my_object_set_bar (MyObject *obj,
  // 		     gboolean bar)
  // {
  //   bar = !!bar;
  //   if (obj->bar != bar)
  //     {
  // 	obj->bar = bar;
  // 	g_object_notify_by_pspec (G_OBJECT (obj), properties[PROP_BAR]);
  //     }
  // }
  
  // static void
  // my_object_set_baz (MyObject  *obj,
  // 				     const gchar *baz)
  // {
  //   if (g_strcmp0 (obj->baz, baz) != 0)
  //     {
  // 	g_free (obj->baz);
  // 	obj->baz = g_strdup (baz);
  // 	g_object_notify_by_pspec (G_OBJECT (obj), properties[PROP_BAZ]);
  //     }
  // }
  
  static void
  my_object_finalize (GObject *gobject)
  {
    //g_free (((MyObject *) gobject)->baz);
    G_OBJECT_CLASS (my_object_parent_class)->finalize (gobject);
  }
  
  static void
  my_object_set_property (GObject *gobject,
			  guint prop_id,
			  const GValue *value,
			  GParamSpec *pspec)
  {
    MyObject *myobj = (MyObject *) gobject;

    GObjectWrapper *context = static_cast <GObjectWrapper *> (myobj->context);
    //context->custom_properties_[prop_id]->invoke_set (value, context->user_data_);
    //context->invoke_set(prop_id, value);
    (*context->get_set_method_pointer (prop_id)) (value, 
						  context->get_user_data (g_param_spec_get_nick (pspec)));

    // switch (prop_id)
    //   {
    //   case PROP_FOO:
    // 	my_object_set_foo (myobj, g_value_get_int (value));
    // 	break;
	
    //   case PROP_BAR:
    // 	my_object_set_bar (myobj, g_value_get_boolean (value));
    // 	break;
	
    //   case PROP_BAZ:
    // 	my_object_set_baz (myobj, g_value_get_string (value));
    // 	break;

    //   case 6:
    // 	my_object_set_foo (myobj, g_value_get_int (value));
    // 	break;

    //   default:
    // 	g_warning ("set_property: property not found %d", prop_id);
    //   }
  }
  
   static void
   my_object_get_property (GObject *gobject,
			   guint prop_id,
			   GValue *value,
			   GParamSpec *pspec)
   {
     MyObject *myobj = (MyObject *) gobject;
     
     GObjectWrapper *context = static_cast <GObjectWrapper *> (myobj->context);
     //context->custom_properties_[prop_id]->invoke_set (value, context->user_data_);
     //context->invoke_set(prop_id, value);
     (*context->get_get_method_pointer (prop_id)) (value, 
						   context->get_user_data (g_param_spec_get_nick (pspec)));
    
     // switch (prop_id)
     //   {
     //   case PROP_FOO:
     // 	 g_value_set_int (value, myobj->foo);
     // 	break;
	
     //   case PROP_BAR:
     // 	 g_value_set_boolean (value, myobj->bar);
     // 	break;
	
     //   case PROP_BAZ:
     // 	 g_value_set_string (value, myobj->baz);
     // 	break;

     // 	case 6:
     // 	 g_value_set_int (value, myobj->foo);
     // 	break;

     //   default:
     // 	 g_warning ("get_property: property not found %d", prop_id);
     //   }
   }

 static void
 my_object_class_init (MyObjectClass *klass)
 {
   GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
   
   // properties[PROP_FOO] = g_param_spec_int ("foo", "Foo", "Foo",
   // 					    -1, G_MAXINT,
   // 					    0,
   // 					    (GParamFlags)G_PARAM_READWRITE);
   // properties[PROP_BAR] = g_param_spec_boolean ("bar", "Bar", "Bar",
   // 						FALSE,
   // 						(GParamFlags)G_PARAM_READWRITE);
   // properties[PROP_BAZ] = g_param_spec_string ("baz", "Baz", "Baz",
   // 					       NULL,
   // 					       (GParamFlags)G_PARAM_READWRITE);
   
   gobject_class->set_property = my_object_set_property;
   gobject_class->get_property = my_object_get_property;
   gobject_class->finalize = my_object_finalize;
   
   //   g_object_class_install_properties (gobject_class, N_PROPERTIES, properties);
 }
  
  static void 
  my_object_init (MyObject *self)
  {
    // self->foo = 42;
    // self->bar = TRUE;
    // self->baz = g_strdup ("Hello");
  }

  // ---------------------------------- CPP CLASS ----------------------------
  //property id 0 is not allowed, starting at 1 
  guint GObjectWrapper::next_prop_id_ = 1; 
  std::map<guint, GObjectCustomProperty::ptr> GObjectWrapper::custom_properties_;

  GObjectWrapper::GObjectWrapper ()
  {
    my_object_ = (MyObject *)g_object_new (my_object_get_type (), NULL);
    my_object_-> context = this;
    default_user_data_ = NULL;
    //GParamSpec *pspec;

    // pspec = g_object_class_find_property (G_OBJECT_GET_CLASS (my_object_), "foo");

    // gint val;
    // g_object_get (my_object_, "foo", &val, NULL);
    // g_print ("foo: %d\n", val);
    // g_object_set (my_object_, "foo", 47, NULL);
    // g_object_get (my_object_, "foo", &val, NULL);
    // g_print ("foo: %d\n", val);

    // GParamSpec *myparam = g_param_spec_int ("coucou", "hey", "truc",
    // 					     -1, G_MAXINT,
    // 					     0,
    // 					     (GParamFlags)G_PARAM_READWRITE);
    // g_object_class_install_property (G_OBJECT_GET_CLASS (my_object_),
    // 				     (guint)6,
    // 				     myparam);

    // g_object_get (my_object_, "coucou", &val, NULL);
    // g_print ("coucou: %d\n", val);
    // g_object_set (my_object_, "coucou", 6, NULL);
    // g_object_get (my_object_, "coucou", &val, NULL);
    // g_print ("coucou: %d\n", val);
  }

  void 
  GObjectWrapper::notify_property_changed (GObject *object, GParamSpec *pspec)
  {
    g_object_notify_by_pspec (object, pspec);
  }

  void
  GObjectWrapper::set_user_data (std::string nickname,
				 void *user_data)
  {
    user_datas_[nickname] = user_data;
  }

  //TODO provide other make_..._property for other types
  //set_method and get_method must be static
  GParamSpec * 
  GObjectWrapper::make_int_property (const gchar *nickname, 
				     const gchar *description,
				     gint min_value,
				     gint max_value,
				     gint default_value,
				     GParamFlags read_write_flags,
				     GObjectCustomProperty::set_method_pointer set_method,
				     GObjectCustomProperty::get_method_pointer get_method)
  {
    guint prop_id = next_prop_id_;
    next_prop_id_++;

    gchar *name = g_strdup_printf ("customprop%d", prop_id);
    g_debug ("custom property internal name %s", name);

    GParamSpec *param = g_param_spec_int (name, 
					  nickname, 
					  description,
					  min_value, 
					  max_value,
					  default_value,
					  read_write_flags);
    
    //FIXME only methods seems to be required, so maybe move other args
    GObjectCustomProperty::ptr property =  
      GObjectCustomProperty::make_custom_property (set_method,
						   get_method);
    
    custom_properties_[prop_id] = property;
    
    //TODO find a way to get CLASS without instanciating an unused object
    MyObject *obj = (MyObject *)g_object_new (my_object_get_type (), NULL);
    g_object_class_install_property (G_OBJECT_GET_CLASS (obj),
				     prop_id,
				     param);
    g_object_unref (obj);
    return param;
  }
  
  //set_method and get_method must be static
  GParamSpec * 
  GObjectWrapper::make_string_property (const gchar *nickname, 
					const gchar *description,
					const gchar *default_value,
					GParamFlags read_write_flags,
					GObjectCustomProperty::set_method_pointer set_method,
					GObjectCustomProperty::get_method_pointer get_method)
  {
    guint prop_id = next_prop_id_;
    next_prop_id_++;
    
    gchar *name = g_strdup_printf ("customprop%d", prop_id);
    g_debug ("custom property internal name %s", name);
    
    GParamSpec *param = g_param_spec_string (name,
					     nickname,
					     description,
					     default_value,
					     read_write_flags);

  GObjectCustomProperty::ptr property =  
    GObjectCustomProperty::make_custom_property (set_method,
						 get_method);
  
  custom_properties_[prop_id] = property;
  
  //TODO find a way to get CLASS without instanciating an unused object
  MyObject *obj = (MyObject *)g_object_new (my_object_get_type (), NULL);
  g_object_class_install_property (G_OBJECT_GET_CLASS (obj),
				   prop_id,
				   param);
  g_object_unref (obj);
  return param;
}


  //set_method and get_method must be static
  GParamSpec * 
  GObjectWrapper::make_boolean_property (const gchar *nickname, 
					 const gchar *description,
					 gboolean default_value,
					 GParamFlags read_write_flags,
					 GObjectCustomProperty::set_method_pointer set_method,
					 GObjectCustomProperty::get_method_pointer get_method)
  {
    guint prop_id = next_prop_id_;
    next_prop_id_++;
    
    gchar *name = g_strdup_printf ("customprop%d", prop_id);
    g_debug ("custom property internal name %s", name);
    
    GParamSpec *param = g_param_spec_boolean (name,
					      nickname,
					      description,
                                              default_value,
					      read_write_flags);

    GObjectCustomProperty::ptr property =  
      GObjectCustomProperty::make_custom_property (set_method,
						   get_method);
    
  custom_properties_[prop_id] = property;
  
  //TODO find a way to get CLASS without instanciating an unused object
  MyObject *obj = (MyObject *)g_object_new (my_object_get_type (), NULL);
  g_object_class_install_property (G_OBJECT_GET_CLASS (obj),
				   prop_id,
				   param);
  g_object_unref (obj);
  return param;
}


  GObject *
  GObjectWrapper::get_gobject ()
  {
    return G_OBJECT (my_object_);
  }
  
  GObjectWrapper::~GObjectWrapper ()
  {
    g_object_unref (my_object_);
  }

  GObjectCustomProperty::set_method_pointer
  GObjectWrapper::get_set_method_pointer (guint prop_id)
  {
    return custom_properties_[prop_id]->set_method_;
  }

  GObjectCustomProperty::get_method_pointer
  GObjectWrapper::get_get_method_pointer (guint prop_id)
  {
    return custom_properties_[prop_id]->get_method_;
  }

  void *
  GObjectWrapper::get_user_data (std::string nickname)
  {
    if (user_datas_.find (nickname) == user_datas_.end ())
      return default_user_data_;
    return user_datas_[nickname];
  }

  void 
  GObjectWrapper::set_default_user_data (void *default_user_data)
  {
    default_user_data_ = default_user_data;
  }
 }
