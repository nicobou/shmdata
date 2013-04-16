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
 * The Param class
 */

#include "switcher/signal-string.h"

namespace switcher
{

  Signal::Signal ()
  {
    json_description_.reset (new JSONBuilder());
  }

  bool
  Signal::set_gobject_signame (GObject *object, std::string signame)
  {
    if (!G_IS_OBJECT (object))
      {
	g_debug ("Signal: object is not a gobject");
	return false;
      }
     
    guint id = g_signal_lookup (signame.c_str (), 
				G_OBJECT_CLASS_TYPE(object));
    if (id == 0)
      {
	g_debug ("Signal: object does not have a signal named %s", 
		 signame.c_str ());
	return false;
      }
    object_ = object;
    id_ = id;
    make_description ();
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
  Signal::make_description ()
  {
  //   /* Signals/Actions Block */
  //   guint *signals;
  //   guint nsignals;
  //   gint i = 0, j, k;
  //   GSignalQuery *query = NULL;
  //   GType type;
  //   GSList *found_signals, *l;
     
      
  //   for (type = G_OBJECT_TYPE (element); type; type = g_type_parent (type)) {
  //     if (type == GST_TYPE_ELEMENT || type == GST_TYPE_OBJECT)
  //       break;

  //     if (type == GST_TYPE_BIN && G_OBJECT_TYPE (element) != GST_TYPE_BIN)
  //       continue;

  //     signals = g_signal_list_ids (type, &nsignals);
  //     for (i = 0; i < nsignals; i++) {
  //       query = g_new0 (GSignalQuery, 1);
  //       g_signal_query (signals[i], query);

  //       if ((k == 0 && !(query->signal_flags & G_SIGNAL_ACTION)) ||
  //           (k == 1 && (query->signal_flags & G_SIGNAL_ACTION)))
  //         found_signals = g_slist_append (found_signals, query);
  //       else
  //         g_free (query);
  //     }
  //     g_free (signals);
  //     signals = NULL;
  //   }

  //   if (found_signals) {
  //     n_print ("\n");
  //     if (k == 0)
  //       n_print ("Element Signals:\n");
  //     else
  //       n_print ("Element Actions:\n");
  //   } else {
  //     continue;
  //   }

  //   for (l = found_signals; l; l = l->next) {
  //     gchar *indent;
  //     int indent_len;

  //     query = (GSignalQuery *) l->data;
  //     indent_len = strlen (query->signal_name) +
  // 	strlen (g_type_name (query->return_type)) + 24;

  //     indent = g_new0 (gchar, indent_len + 1);
  //     memset (indent, ' ', indent_len);

  //     n_print ("  \"%s\" :  %s user_function (%s* object",
  // 	       query->signal_name,
  // 	       g_type_name (query->return_type), g_type_name (type));

  //     for (j = 0; j < query->n_params; j++) {
  //       if (_name)
  //         g_print ("%s", _name);
  //       if (G_TYPE_IS_FUNDAMENTAL (query->param_types[j])) {
  //         g_print (",\n%s%s arg%d", indent,
  // 		   g_type_name (query->param_types[j]), j);
  //       } else if (G_TYPE_IS_ENUM (query->param_types[j])) {
  //         g_print (",\n%s%s arg%d", indent,
  // 		   g_type_name (query->param_types[j]), j);
  //       } else {
  //         g_print (",\n%s%s* arg%d", indent,
  // 		   g_type_name (query->param_types[j]), j);
  //       }
  //     }

  //     if (k == 0) {
  //       if (_name)
  //         g_print ("%s", _name);
  //       g_print (",\n%sgpointer user_data);\n", indent);
  //     } else
  //       g_print (");\n");

  //     g_free (indent);
  //   }

  //   if (found_signals) {
  //     g_slist_foreach (found_signals, (GFunc) g_free, NULL);
  //     g_slist_free (found_signals);
  //   }
  }
}
 
