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

#include "create-remove-spy.h"
#include "gst-utils.h"

namespace switcher
{

  QuiddityDocumentation CreateRemoveSpy::doc_  ("spy", "create_remove_spy",				
						"spy manager for quidity creation and removal and convert into signals");


  bool
  CreateRemoveSpy::init ()
  {
    // QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    // if ((bool) manager)
    //   {
    //    	if (!manager->set_created_hook (CreateRemoveSpy::on_created, this))
    //    	  return false;
    //    	if (!manager->set_removed_hook (CreateRemoveSpy::on_removed, this))
    //    	  return false;
    //   }
    // else
    //   return false;

    //   set_name ("spy");
    //  //we got the hook, so make signals of it
    //    GType types[1];
    //    types[0] = G_TYPE_STRING;
    //    guint id_created = GObjectWrapper::make_signal (G_TYPE_NONE,
    //    						    1,
    //    						    types);    
    //     guint id_removed = GObjectWrapper::make_signal (G_TYPE_NONE,
    //     						    1,
    //     						    types);    
    
    //    GObjectWrapper::ptr gobject = get_quiddity_internal_gobject ();
    //    register_signal_gobject_by_id ("on-quiddity-created",
    // 				      gobject->get_gobject (), 
    // 				      id_created);
      // set_signal_description ("on-quiddity-created",
      // 			    "a quiddity has been created",
      // 			    Signal::make_arg_description("nick_name",
      // 							 "the quiddity nick name",
      // 							 NULL));
     // register_signal_gobject_by_id ("on-quiddity-removed",
     // 				   gobject->get_gobject (), 
     // 				   id_removed);
     // set_signal_description ("on-quiddity-removed",
     // 			    "a quiddity has been removed",
     // 			    Signal::make_arg_description("nick_name",
     // 							 "the quiddity nick name",
     // 							 NULL));

      return true;
  }
  
  CreateRemoveSpy::~CreateRemoveSpy ()
  {
    QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    if ((bool) manager)
      manager->reset_create_remove_hooks ();
  }
  
  void 
  CreateRemoveSpy::on_created (std::string quiddity_nick_name, void *user_data)
  {
    CreateRemoveSpy *context = static_cast<CreateRemoveSpy *> (user_data);
    context->signal_emit ("on-quiddity-created", quiddity_nick_name.c_str ());
  }

  void 
  CreateRemoveSpy::on_removed (std::string quiddity_nick_name, void *user_data)
  {
    CreateRemoveSpy *context = static_cast<CreateRemoveSpy *> (user_data);
    context->signal_emit ("on-quiddity-removed", quiddity_nick_name.c_str ());
  }

  QuiddityDocumentation 
  CreateRemoveSpy::get_documentation ()
  {
    return doc_;
  }
  
}
