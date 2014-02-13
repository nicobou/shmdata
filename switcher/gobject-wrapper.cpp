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
  } MyObject;

  typedef struct _MyObjectClass {
    GObjectClass parent_class;
  } MyObjectClass;

  static GType my_object_get_type (void);
  G_DEFINE_TYPE (MyObject, my_object, G_TYPE_OBJECT);

  static void
  my_object_finalize (GObject *gobject)
  {
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
    (*context->get_set_method_pointer (prop_id)) (value, 
						  context->property_get_user_data (g_param_spec_get_nick (pspec)));
  }
  
   static void
   my_object_get_property (GObject *gobject,
			   guint prop_id,
			   GValue *value,
			   GParamSpec *pspec)
   {
     MyObject *myobj = (MyObject *) gobject;
     GObjectWrapper *context = static_cast <GObjectWrapper *> (myobj->context);
     (*context->get_get_method_pointer (prop_id)) (value, 
						   context->property_get_user_data (g_param_spec_get_nick (pspec)));
   }

 static void
 my_object_class_init (MyObjectClass *klass)
 {
   GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
   gobject_class->set_property = my_object_set_property;
   gobject_class->get_property = my_object_get_property;
   gobject_class->finalize = my_object_finalize;
 }
  
  static void 
  my_object_init (MyObject */*self*/)
  {}

  // ---------------------------------- CPP CLASS ----------------------------
  //property id 0 is not allowed, starting at 1 
  guint GObjectWrapper::next_prop_id_ = 1; 
  std::map<guint, GObjectCustomProperty::ptr> GObjectWrapper::custom_properties_;

  //signals
  guint GObjectWrapper::next_signal_num_ = 1; 
  //std::map<guint, GObjectCustomSignal::ptr> GObjectWrapper::custom_signals_;

  GObjectWrapper::GObjectWrapper ()
  {
    my_object_ = (MyObject *)g_object_new (my_object_get_type (), NULL);
    my_object_-> context = this;
    property_default_user_data_ = NULL;
    
  }

  bool 
  GObjectWrapper::notify_property_changed (GObject *object, GParamSpec *pspec)
  {
    if (!G_IS_OBJECT (object))
      return false;
    g_object_notify_by_pspec (object, pspec);
    return true;
  }

  void
  GObjectWrapper::property_set_user_data (std::string nickname,
					  void *user_data)
  {
    property_user_datas_[nickname] = user_data;
  }

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

  GParamSpec * 
  GObjectWrapper::make_double_property (const gchar *nickname, 
					const gchar *description,
					gdouble min_value,
					gdouble max_value,
					gdouble default_value,
					GParamFlags read_write_flags,
					GObjectCustomProperty::set_method_pointer set_method,
					GObjectCustomProperty::get_method_pointer get_method)
  {
    guint prop_id = next_prop_id_;
    next_prop_id_++;

    gchar *name = g_strdup_printf ("customprop%d", prop_id);
    g_debug ("custom property internal name %s", name);

    GParamSpec *param = g_param_spec_double (name, 
					     nickname, 
					     description,
					     min_value, 
					     max_value,
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

  GParamSpec *
  GObjectWrapper::make_enum_property (const gchar *nickname, 
				      const gchar *description,
				      const gint default_value, 
				      const GEnumValue *custom_enum,
				      GParamFlags read_write_flags,
				      GObjectCustomProperty::set_method_pointer set_method,
				      GObjectCustomProperty::get_method_pointer get_method)
  {
    guint prop_id = next_prop_id_;
    next_prop_id_++;
    gchar *name = g_strdup_printf ("customprop%d", prop_id);
    
    //  static GEnumValue string_map_enum [1024];
    //   gint gint_default_value = 0;
    //   gint i = 0;
    //   for (auto &it : string_map)
    //     {
    //    	string_map_enum [i].value = i + 1;
    //    	string_map_enum [i].value_name = g_strdup (it.first.c_str ());
    //    	string_map_enum [i].value_nick = g_strdup (it.second.c_str ());
    //    	if (g_strcmp0 (it.first.c_str (), default_value) == 0)
    //    	  gint_default_value = i + 1 ;
    //    	i ++;
    //     }
    //   string_map_enum [i].value = 0;
    //   string_map_enum [i].value_name = NULL;
    //   string_map_enum [i].value_nick = NULL;

    //registering the type with the name calculated previously
    GType gtype = g_enum_register_static (name, custom_enum);
      
    
     GParamSpec *param = g_param_spec_enum (name,
     					   nickname,
     					   description, 
     					   gtype,
     					   default_value,  
     					   (GParamFlags) (read_write_flags // | G_PARAM_STATIC_STRINGS
     							  ));

    

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
    
    gchar *name = g_strdup_printf ("customprop%u", prop_id);
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

  guint
  GObjectWrapper::make_signal (GType return_type,
			       guint n_params,
			       GType *param_types)
  {
    guint sig_id = next_signal_num_;
    next_signal_num_++;
    gchar *name = g_strdup_printf ("custom_signal_%d", sig_id);

    //TODO find a way to get CLASS without instanciating an unused object
     MyObject *obj = (MyObject *)g_object_new (my_object_get_type (), NULL);
     guint signal_id =
       g_signal_newv (name,
    		     G_TYPE_FROM_CLASS (G_OBJECT_GET_CLASS (obj)),
    		     G_SIGNAL_RUN_LAST,
    		     0,
    		     NULL, //GSignalAccumulator
    		     NULL, //gpointer accu_data
    		     NULL, //GSignalCMarshaller
    		     return_type,
    		     n_params,
    		     param_types);
     g_object_unref (obj);
    
    return signal_id;
  }

  guint
  GObjectWrapper::make_signal_action (GClosure *class_closure,
				      GType return_type,
				      guint n_params,
				      GType *param_types)
  {
    guint sig_id = next_signal_num_;
    next_signal_num_++;
    gchar *name = g_strdup_printf ("custom_signal_%d", sig_id);

   //TODO find a way to get CLASS without instanciating an unused object
    MyObject *obj = (MyObject *)g_object_new (my_object_get_type (), NULL);
    guint signal_id =
      g_signal_newv (name,
		     G_TYPE_FROM_CLASS (G_OBJECT_GET_CLASS (obj)),
		     (GSignalFlags)(G_SIGNAL_RUN_LAST | G_SIGNAL_ACTION),
		     class_closure,
		     NULL, //GSignalAccumulator
		     NULL, //gpointer accu_data
		     NULL, //GSignalCMarshaller
		     return_type,
		     n_params,
		     param_types);
    g_object_unref (obj);
    
    // GObjectCustomSignal::ptr signal =  
    //   GObjectCustomSignal::make_custom_signal ();
    
    // custom_signals_[signal_id] = signal;
    return signal_id;
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
  GObjectWrapper::property_get_user_data (std::string nickname)
  {
    if (property_user_datas_.find (nickname) == property_user_datas_.end ())
      return property_default_user_data_;
    return property_user_datas_[nickname];
  }

  void 
  GObjectWrapper::property_set_default_user_data (void *default_user_data)
  {
    property_default_user_data_ = default_user_data;
  }

  bool 
  GObjectWrapper::is_property_nickname_taken (std::string nickname)
  {
    return property_user_datas_.find (nickname) != property_user_datas_.end ();
  }
 }
