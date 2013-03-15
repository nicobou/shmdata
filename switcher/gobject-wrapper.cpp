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

#include "switcher/gobject-wrapper.h"

namespace switcher
{



  //gobject
  typedef struct _MyObject {
    GObject parent_instance;
    gint foo;
    gboolean bar;
    gchar *baz;
  } MyObject;

  typedef struct _MyObjectClass {
    GObjectClass parent_class;
  } MyObjectClass;

  static GType my_object_get_type (void);
  enum { PROP_0, PROP_FOO, PROP_BAR, PROP_BAZ, N_PROPERTIES };
  static GParamSpec *properties[N_PROPERTIES] = { NULL, };
  G_DEFINE_TYPE (MyObject, my_object, G_TYPE_OBJECT);

  static void
  my_object_set_foo (MyObject *obj,
		     gint foo)
  {
    if (obj->foo != foo)
      {
	obj->foo = foo;
	//g_object_notify_by_pspec (G_OBJECT (obj), properties[PROP_FOO]);
	g_object_notify_by_pspec (G_OBJECT (obj), 
				  g_object_class_find_property (G_OBJECT_GET_CLASS (obj), "coucou"));
      }
  }
  
  static void
  my_object_set_bar (MyObject *obj,
				     gboolean bar)
  {
    bar = !!bar;
    if (obj->bar != bar)
      {
	obj->bar = bar;
	g_object_notify_by_pspec (G_OBJECT (obj), properties[PROP_BAR]);
      }
  }
  
  static void
  my_object_set_baz (MyObject  *obj,
				     const gchar *baz)
  {
    if (g_strcmp0 (obj->baz, baz) != 0)
      {
	g_free (obj->baz);
	obj->baz = g_strdup (baz);
	g_object_notify_by_pspec (G_OBJECT (obj), properties[PROP_BAZ]);
      }
  }
  
  static void
  my_object_finalize (GObject *gobject)
  {
    g_free (((MyObject *) gobject)->baz);
    G_OBJECT_CLASS (my_object_parent_class)->finalize (gobject);
  }
  
  static void
  my_object_set_property (GObject *gobject,
			  guint prop_id,
			  const GValue *value,
			  GParamSpec *pspec)
  {
    MyObject *myobj = (MyObject *) gobject;
    
    switch (prop_id)
      {
      case PROP_FOO:
	my_object_set_foo (myobj, g_value_get_int (value));
	break;
	
      case PROP_BAR:
	my_object_set_bar (myobj, g_value_get_boolean (value));
	break;
	
      case PROP_BAZ:
	my_object_set_baz (myobj, g_value_get_string (value));
	break;

      case 6:
	my_object_set_foo (myobj, g_value_get_int (value));
	break;

      default:
	g_warning ("set_property: property not found %d", prop_id);
      }
  }
  
   static void
   my_object_get_property (GObject *gobject,
			   guint prop_id,
			   GValue *value,
			   GParamSpec *pspec)
   {
     MyObject *myobj = (MyObject *) gobject;
     
     switch (prop_id)
       {
       case PROP_FOO:
	 g_value_set_int (value, myobj->foo);
   	break;
	
       case PROP_BAR:
	 g_value_set_boolean (value, myobj->bar);
   	break;
	
       case PROP_BAZ:
	 g_value_set_string (value, myobj->baz);
   	break;

	case 6:
	 g_value_set_int (value, myobj->foo);
   	break;

       default:
	 g_warning ("get_property: property not found %d", prop_id);
       }
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
    self->foo = 42;
    self->bar = TRUE;
    self->baz = g_strdup ("Hello");
  }

  // ---------------------------------- CPP CLASS ---------------------------------------

  int GObjectWrapper::next_prop_id_ = 79; 

  GObjectWrapper::GObjectWrapper ()
  {
    my_object_ = (MyObject *)g_object_new (my_object_get_type (), NULL);
    GParamSpec *pspec;

    // pspec = g_object_class_find_property (G_OBJECT_GET_CLASS (my_object_), "foo");

    gint val;
    // g_object_get (my_object_, "foo", &val, NULL);
    // g_print ("foo: %d\n", val);
    // g_object_set (my_object_, "foo", 47, NULL);
    // g_object_get (my_object_, "foo", &val, NULL);
    // g_print ("foo: %d\n", val);

    GParamSpec *myparam = g_param_spec_int ("coucou", "hey", "truc",
					     -1, G_MAXINT,
					     0,
					     (GParamFlags)G_PARAM_READWRITE);
    g_object_class_install_property (G_OBJECT_GET_CLASS (my_object_),
				     (guint)6,
				     myparam);

    g_object_get (my_object_, "coucou", &val, NULL);
    g_print ("coucou: %d\n", val);
    g_object_set (my_object_, "coucou", 6, NULL);
    g_object_get (my_object_, "coucou", &val, NULL);
    g_print ("coucou: %d\n", val);
  }
  
  bool 
  GObjectWrapper::install_int_property (std::string nickname, 
					std::string description)
  {
    
    
    next_prop_id_++;
  }
  


  GObjectWrapper::~GObjectWrapper ()
  {
    g_object_unref (my_object_);
  }


 }
