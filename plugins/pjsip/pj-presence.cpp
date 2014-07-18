/*
 * This file is part of switcher-pjsip.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#include "pj-presence.h"
#include "pj-sip.h"

namespace switcher
{

  PJPresence::PJPresence (PJSIP *sip_instance) :
    sip_instance_ (sip_instance),
    account_id_ (-1),
    registration_mutex_ (),
    registration_cond_ ()
  {
    sip_instance_->install_method ("Register SIP Account", //long name
				   "register", //name
				   "register a SIP account", //description
				   "success", //return description
				   Method::make_arg_description ("SIP User Name", //first arg long name
								 "user", //fisrt arg name
								 "string", //first arg description
								 "SIP Domain", 
								 "domain", 
								 "string", 
								 "SIP password", 
								 "password", 
								 "string", 
								 NULL),
				   (Method::method_ptr) &register_account_wrapped, 
				   G_TYPE_BOOLEAN,
				   Method::make_arg_type_description (G_TYPE_STRING, 
								      G_TYPE_STRING, 
								      G_TYPE_STRING, 
								      NULL),
				   this);

  }

  PJPresence::~PJPresence ()
  {}

  gboolean
  PJPresence::register_account_wrapped (gchar *user, gchar *domain, gchar *password, void *user_data)
  {
    PJPresence *context = static_cast<PJPresence *> (user_data);
    if (NULL == user || NULL == domain || NULL == password)
      {
	g_warning ("register sip account received NULL user or domain or password");
	return FALSE;
      }
    context->sip_instance_->run_command_sync (std::bind (&PJPresence::register_account, 
							 context, 
							 std::string (user),
							 std::string (domain),
							 std::string (password)));
    return TRUE;
  }

  void 
  PJPresence::register_account (const std::string &sip_user, 
			   const std::string &sip_domain, 
			   const std::string &sip_password)
  {
    std::unique_lock<std::mutex> lock (registration_mutex_);

    // Register to SIP server by creating SIP account.
    pjsua_acc_config cfg;
    pj_status_t status;

    gchar *id = g_strdup_printf ("sip:%s@%s",  
				 sip_user.c_str (),
				 sip_domain.c_str ());
    gchar *reg_uri = g_strdup_printf ("sip:%s", sip_domain.c_str ());
    gchar *user = g_strdup (sip_user.c_str ());
    gchar *domain = g_strdup (sip_domain.c_str ());
    gchar *digest = g_strdup ("digest");
    gchar *password = g_strdup (sip_password.c_str ());
    pjsua_acc_config_default (&cfg);
    cfg.id = pj_str (id); 
    cfg.reg_uri = pj_str (reg_uri);
    cfg.cred_count = 1;
    cfg.cred_info[0].realm = pj_str (domain);
    cfg.cred_info[0].scheme = pj_str (digest);
    cfg.cred_info[0].username = pj_str (user);
    cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
    cfg.cred_info[0].data = pj_str (password);
    cfg.publish_enabled = PJ_TRUE; 
    cfg.register_on_acc_add = PJ_TRUE; //or  pjsua_acc_set_registration (account_id_, PJ_TRUE);

    status = pjsua_acc_add (&cfg, PJ_TRUE, &account_id_);
    g_free (id);
    g_free (reg_uri);
    g_free (user);
    g_free (domain);
    g_free (digest);
    g_free (password);
    if (status != PJ_SUCCESS) 
      {
	account_id_ = -1;
	return;
      }
    pjsua_acc_set_user_data (account_id_, this);
    registration_cond_.wait (lock);

    add_buddy ("sip:1000@10.10.30.252");
    add_buddy ("sip:1001@10.10.30.252");
    add_buddy ("sip:1002@10.10.30.252");
    add_buddy ("sip:1003@10.10.30.252");
    add_buddy ("sip:1004@10.10.30.252");
    add_buddy ("sip:1005@10.10.30.252");
    add_buddy ("sip:1006@10.10.30.252");
    add_buddy ("sip:1007@10.10.30.252");
    add_buddy ("sip:1008@10.10.30.252");
    add_buddy ("sip:1009@10.10.30.252");
    add_buddy ("sip:1010@10.10.30.252");
    add_buddy ("sip:1011@10.10.30.252");
    add_buddy ("sip:1012@10.10.30.252");
    add_buddy ("sip:1013@10.10.30.252");
    add_buddy ("sip:1014@10.10.30.252");
    add_buddy ("sip:1015@10.10.30.252");
    add_buddy ("sip:1016@10.10.30.252");
    add_buddy ("sip:1017@10.10.30.252");
    add_buddy ("sip:1018@10.10.30.252");
    add_buddy ("sip:1019@10.10.30.252");
  }

  void
  PJPresence::add_buddy (const std::string &sip_user)
  {
    pjsua_buddy_config buddy_cfg;
    pjsua_buddy_id buddy_id;
    pj_status_t status = PJ_SUCCESS;
    
    if (pjsua_verify_url(sip_user.c_str ()) != PJ_SUCCESS) {
      g_warning ("Invalid buddy URI %s", sip_user.c_str ());
      return;
    } 

    pj_bzero(&buddy_cfg, sizeof(pjsua_buddy_config));
    gchar *buddy = g_strdup (sip_user.c_str ());
    buddy_cfg.uri = pj_str(buddy);
    buddy_cfg.subscribe = PJ_TRUE;
    buddy_cfg.user_data = this;
    status = pjsua_buddy_add(&buddy_cfg, &buddy_id);
    if (status == PJ_SUCCESS) 
      g_debug ("Buddy added");
    g_free (buddy);
  }

  void 
  PJPresence::on_registration_state (pjsua_acc_id acc_id, pjsua_reg_info *info)
  {
    PJPresence *context = static_cast<PJPresence *> (pjsua_acc_get_user_data (acc_id));
    std::unique_lock <std::mutex> lock (context->registration_mutex_);
    if (PJ_SUCCESS != info->cbparam->status)
      {
	if (-1 != context->account_id_)
	  {
	    pj_status_t status = pjsua_acc_del (context->account_id_);
	    if (PJ_SUCCESS != status) 
	      g_warning ("Error deleting account after registration failled");
	    context->account_id_ = -1;
	  }
      }
    g_print ("registration SIP status code %d\n", info->cbparam->code);
    context->registration_cond_.notify_one ();
  }

  void 
  PJPresence::on_buddy_state(pjsua_buddy_id buddy_id)
  {
    PJPresence *context = static_cast<PJPresence *> (pjsua_buddy_get_user_data (buddy_id));
    if (NULL == context)
      return;
    pjsua_buddy_info info;
    pjsua_buddy_get_info(buddy_id, &info);

    std::string activity;
    if (PJRPID_ACTIVITY_UNKNOWN == info.rpid.activity)
      activity = "unknown";
    if (PJRPID_ACTIVITY_AWAY == info.rpid.activity)
      activity = "away";
    if (PJRPID_ACTIVITY_BUSY == info.rpid.activity)
      activity = "busy";
    
    
    g_print ("%.*s status is %.*s, subscription state is %s "
	     "(last termination reason code=%d %.*s)\n"
	     "rpid  activity %s, note %.*s\n",
	     (int)info.uri.slen,
	     info.uri.ptr,
	     (int)info.status_text.slen,
	     info.status_text.ptr,
	     info.sub_state_name,
	     info.sub_term_code,
	     (int)info.sub_term_reason.slen,
	     info.sub_term_reason.ptr,
	     activity.c_str (),
	     (int)info.rpid.note.slen,
	     info.rpid.note.ptr);
    
    data::Tree::ptr tree = data::make_tree ();
    
    std::string buddy_url (info.uri.ptr,
			   (size_t)info.uri.slen);
    tree->graft (".sip_url", data::make_tree (buddy_url));

    std::string status ("unknown");

     switch (info.status) {
     case PJSUA_BUDDY_STATUS_UNKNOWN :
       break;
     case PJSUA_BUDDY_STATUS_ONLINE :
       status = "online";
       break;
     case PJSUA_BUDDY_STATUS_OFFLINE :
       status = "offline";
       break;
     default:
       break;
     }
     if (PJRPID_ACTIVITY_AWAY == info.rpid.activity)
       status = "away";
     if (PJRPID_ACTIVITY_BUSY == info.rpid.activity)
       status = "busy";

     tree->graft (".status", data::make_tree (status));

     tree->graft (".status_text", 
		  data::make_tree (std::string (info.status_text.ptr, 
						(size_t)info.status_text.slen)));
     tree->graft (".subscription_state", 
		  data::make_tree (std::string (info.sub_state_name)));
     context->sip_instance_->graft_tree (std::string (".presence." + buddy_url), tree);
  }
 
}
