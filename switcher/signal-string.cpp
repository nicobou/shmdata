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
 * The Signal class that wraps gobject signals and add some documentation to it
 */

#include "signal-string.h"
#include "gst-utils.h"
#include <algorithm>

namespace switcher
{

  Signal::Signal ()
  {
    json_description_.reset (new JSONBuilder());
    hook_id_ = 0;
  }

  Signal::~Signal ()
  {
    g_signal_remove_emission_hook (id_, hook_id_);
  }

  bool
  Signal::set_gobject_signame (GObject *object, 
			       std::string gobject_signal_name)
  {
    if (!G_IS_OBJECT (object))
      {
	g_debug ("Signal: object is not a gobject");
	return false;
      }
     
    guint id = g_signal_lookup (gobject_signal_name.c_str (), 
				G_TYPE_FROM_CLASS (G_OBJECT_CLASS_TYPE(object)));
    if (id == 0)
      {
	g_debug ("Signal: object does not have a signal named %s", 
		 gobject_signal_name.c_str ());
	return false;
      }

    return set_gobject_sigid (object, id);
  }

  bool
  Signal::set_gobject_sigid (GObject *object, 
			     guint gobject_signal_id)
  {
    if (!G_IS_OBJECT (object))
      {
    	g_debug ("Signal: object is not a gobject");
    	return false;
      }

    object_ = object;
    id_ = gobject_signal_id;
    inspect_gobject_signal ();
    hook_id_ = g_signal_add_emission_hook (id_, 0, on_signal_emitted, this, NULL);

    return true;
  }
   
  std::string
  Signal::get_description ()
  {
    return json_description_->get_string(true);
  }

  JSONBuilder::Node
  Signal::get_json_root_node ()
  {
    return json_description_->get_root ();
  }

  //make json formated description 
  void
  Signal::inspect_gobject_signal ()
  {
    /* Signals/Actions Block */
    //guint *signals;
    //guint nsignals;
    guint j;
    GSignalQuery *query = NULL;
    //GType type;
    
    query = g_new0 (GSignalQuery, 1);
    g_signal_query (id_, query);
    
    if (query->signal_flags & G_SIGNAL_ACTION)
      is_action_ = TRUE;
    else
      is_action_ = FALSE;
    
    return_type_ = query->return_type;
    
    for (j = 0; j < query->n_params; j++) {
      arg_types_.push_back (query->param_types[j]);
    }
    g_free (query);
  }
  
 void
 Signal::set_description (std::string long_name,
			  std::string signal_name,
			  std::string short_description,
			  std::string return_description,
			  args_doc arg_description)
  {
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("long name", long_name.c_str ());
    json_description_->add_string_member ("name", signal_name.c_str ());
    json_description_->add_string_member ("description", short_description.c_str ());
    
    if (is_action_)
      json_description_->add_string_member ("type", "action");
    else
      json_description_->add_string_member ("type", "signal");
    json_description_->add_string_member ("return type", g_type_name (return_type_));
    json_description_->add_string_member ("return description", return_description.c_str ());
    json_description_->set_member_name ("arguments");
    json_description_->begin_array ();
    args_doc::iterator it;
    int j=0;
    if (!arg_description.empty ())
      for (it = arg_description.begin() ; it != arg_description.end(); it++ )
	{
	  json_description_->begin_object ();
	  json_description_->add_string_member ("long name", std::get<0>(*it).c_str ());
	  json_description_->add_string_member ("name",std::get<1>(*it).c_str ());
	  json_description_->add_string_member ("description",std::get<2>(*it).c_str ());
	  json_description_->add_string_member ("type",g_type_name (arg_types_[j])); 
	  json_description_->end_object ();
	}
    json_description_->end_array ();
    json_description_->end_object ();
  }

 std::vector<GType> 
   Signal::make_arg_type_description (GType first_arg_type, ...)
   {
     std::vector<GType> res;
     GType arg_type;
     va_list vl;
     va_start(vl, first_arg_type);
     res.push_back (first_arg_type);
     while ( (arg_type = va_arg( vl, GType)) )
       res.push_back (arg_type);
     va_end(vl);
     return res;
   }

  Signal::args_doc
  Signal::make_arg_description (const gchar *first_arg_long_name, ...)
  {
    args_doc res;
    va_list vl;
    char *arg_long_name;
    char *arg_name;
    char *arg_desc;
    va_start(vl, first_arg_long_name);
    if (g_strcmp0 (first_arg_long_name, "none") != 0 
	&& (arg_name = va_arg(vl, char *)) 
	&& (arg_desc = va_arg(vl, char *)))
      res.push_back (std::make_tuple (first_arg_long_name, 
				      arg_name,
				      arg_desc));
    
    gboolean parsing = true;
    do
      {
	arg_long_name = va_arg( vl, char *);
	if (arg_long_name != NULL)
	  {
	    arg_name = va_arg( vl, char *); 
	    arg_desc = va_arg( vl, char *);
	
	    if (arg_name != NULL && arg_desc != NULL)
	      res.push_back (std::make_tuple (arg_long_name, 
					      arg_name,
					      arg_desc));
	    else
	      parsing=false;
	  }
	else
	  parsing=false;
      }
    while (parsing);
    
    va_end(vl);
    return res;
    // args_doc res;
    // va_list vl;
    // char *arg_long_name;
    // char *arg_name;
    // char *arg_desc;
    // va_start(vl, first_arg_long_name);
    // if (g_strcmp0 (first_arg_long_name, "none") != 0 
    // 	&& (arg_name = va_arg(vl, char *)) 
    // 	&& (arg_desc = va_arg(vl, char *)))
    //   {
    // 	res.push_back (std::make_tuple (first_arg_long_name, 
    // 					arg_name,
    // 					arg_desc));
    //   }
    // while ((arg_long_name = va_arg( vl, char *))
    // 	   && (arg_name = va_arg( vl, char *)) 
    // 	   && (arg_desc = va_arg( vl, char *)))
    //   {
    // 	res.push_back (std::make_tuple (arg_long_name, 
    // 					arg_name,
    // 					arg_desc));
    //   }
    // va_end(vl);
    // return res;
  }
  
  gboolean
  Signal::on_signal_emitted (GSignalInvocationHint *,
			     guint n_param_values,
			     const GValue *param_values,
			     gpointer user_data)
  {
     Signal *context = static_cast<Signal *> (user_data);
     GObject *object = (GObject *)g_value_peek_pointer (&param_values[0]);
     if (object != context->object_)
       return TRUE;

     std::vector<std::string> params;
     // g_debug ("signal name n_value %d, object type %s\n", n_param_values, G_OBJECT_TYPE_NAME (object));
     guint i;
     for (i = 1; i < n_param_values; i++) //we do not deserialize the gobject
       {
	 gchar *val_str = GstUtils::gvalue_serialize (&param_values[i]);
	 if (val_str == NULL)//gst-streamer connt serialize this
	   val_str = g_strdup ("NULL");
	 params.push_back (val_str);
	 g_free (val_str);
       }
     std::vector<std::pair<OnEmittedCallback, void *> >::iterator it;
     
     for (it = context->subscribed_on_emitted_callbacks_.begin ();
	  it != context->subscribed_on_emitted_callbacks_.end ();
	  it++)
       it->first (params, it->second);
     
     return TRUE; //keep the hook alive
  }

  bool
  Signal::subscribe (OnEmittedCallback cb, void *user_data)
  {
    std::pair <OnEmittedCallback, void *> cb_pair = std::make_pair (cb, user_data);
    //FIXME do not save twice the same cb/user data for signals 
    // if(std::find(subscribed_on_emitted_callbacks_.begin(), 
    //  		 subscribed_on_emitted_callbacks_.end(), 
    //  		 cb_pair) != subscribed_on_emitted_callbacks_.end()) 
    {
      subscribed_on_emitted_callbacks_.push_back (cb_pair);
      return true;
    } 
    // else
    //   return false;
  }

  bool
  Signal::unsubscribe (OnEmittedCallback cb, void *user_data)
  {
    std::pair <OnEmittedCallback, void *> cb_pair = std::make_pair (cb, user_data);
    std::vector<std::pair<OnEmittedCallback, void *> >::iterator it;
    it = std::find(subscribed_on_emitted_callbacks_.begin(), 
		   subscribed_on_emitted_callbacks_.end(), 
		   cb_pair);
    if(it != subscribed_on_emitted_callbacks_.end())
      {
	subscribed_on_emitted_callbacks_.erase (it);
	return true;
      } 
    else
      return false;
  }

  void 
  Signal::signal_emit (/*GMainContext *context,*/ const gchar */*unused_string*/, va_list  var_args)
  {
    g_signal_emit_valist (object_, id_, 0, var_args);
    // EmitArgs *args = new EmitArgs;
    // args->object_ = object_;
    // args->id_ = id_;
    // va_copy (args->var_args_, var_args);
    // GstUtils::g_idle_add_full_with_context (context, 
    // 					    G_PRIORITY_DEFAULT_IDLE,
    // 					    GSourceFunc (signal_emit_in_main_loop),
    // 					    (gpointer)args,
    // 					    NULL);
  }  
  
  // gboolean 
  // Signal::signal_emit_in_main_loop (gpointer user_data)
  // {
  //   EmitArgs *args = static_cast <EmitArgs *> (user_data);
  //   //args->object_;args->id_;args->var_args_;
  //   g_signal_emit_valist (args->object_, args->id_, 0, args->var_args_);
  //   delete args;
  //   return FALSE;
  // }

  GValue 
  Signal::action_emit (std::vector<std::string> args)
  {
    
    GValue result_value = G_VALUE_INIT;

    if (!is_action_)
      {
	g_warning ("Signal::invoke cannot invoke if the signal is not an action");
	return result_value;
      }

    if (args.size () != arg_types_.size () && arg_types_[0] != G_TYPE_NONE)
      {
	g_warning ("Signal::invoke number of arguments does not correspond to the size of argument types");
	return result_value;
      }

    gsize param_size = 1;
    if (arg_types_[0] != G_TYPE_NONE)
      param_size = arg_types_.size () + 1;

    GValue params[param_size]; //1 is instance and return value
  
    params[0] = G_VALUE_INIT;
    g_value_init (&params[0], G_OBJECT_TYPE (object_));
    g_value_set_object (&params[0], object_);

    //with args
    if (arg_types_[0] != G_TYPE_NONE)
      for (gsize i = 0; i < arg_types_.size (); i++)
	{
	  params[i + 1] = G_VALUE_INIT;
	  g_value_init (&params[i + 1],arg_types_[i]);
	  
	  if (!gst_value_deserialize (&params[i + 1],args[i].c_str()))
	    {
	      g_warning ("Signal::invoke string not transformable into gvalue (argument: %s) ",
			 args[i].c_str());
	      return result_value;
	    }
	}
    
    g_value_init (&result_value, return_type_);
    
    g_signal_emitv (params,
		    id_,
		    0,
		    &result_value);

    for (gsize i = 0; i < param_size ; i++)
      g_value_unset (&params[i]);
    return result_value;
  } 
}
