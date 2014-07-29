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
#include "switcher/scope-exit.h"
namespace switcher
{
  
  GEnumValue PJPresence::status_enum_[8] =  
    { 
      { PJPresence::AVAILABLE, "Available","AVAILABLE" },
      { PJPresence::BUSY, "Busy", "BUSY"},
      { PJPresence::OTP, "On the phone", "OTP"},
      { PJPresence::IDLE, "Idle", "IDLE"},
      { PJPresence::AWAY, "Away", "AWAY"},
      { PJPresence::BRB, "Be right back", "BRB"},
      { PJPresence::OFFLINE, "Offline", "OFFLINE"},
      { 0, nullptr, nullptr}
    };
  
  PJPresence::PJPresence (PJSIP *sip_instance) :
    sip_instance_ (sip_instance),
    account_id_ (-1),
    registration_mutex_ (),
    registration_cond_ (),
    status_enum_spec_ (nullptr),
    status_ (PJPresence::OFFLINE),
    custom_status_spec_ (nullptr),
    custom_status_ (),
    sip_local_user_ ()
  {
    //registering account
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
								 nullptr),
				   (Method::method_ptr) &register_account_wrapped, 
				   G_TYPE_BOOLEAN,
				   Method::make_arg_type_description (G_TYPE_STRING, 
								      G_TYPE_STRING, 
								      G_TYPE_STRING, 
								      nullptr),
				   this);

    sip_instance_->install_method ("Unregister SIP Account", //long name
				   "unregister", //name
				   "unregister SIP account", //description
				   "success", //return description
				   Method::make_arg_description ("none", 
								 nullptr),
				   (Method::method_ptr) &unregister_account_wrapped, 
				   G_TYPE_BOOLEAN,
				   Method::make_arg_type_description (G_TYPE_NONE,
								      nullptr),
				   this);

    //online status
    status_enum_spec_ = 
      sip_instance_->custom_props_->make_enum_property ("status", 
							"Possible Status",
							status_, 
							status_enum_,
							(GParamFlags) G_PARAM_READWRITE,
							PJPresence::set_status,
							PJPresence::get_status,
							this);
    
    sip_instance_->install_property_by_pspec (sip_instance_->custom_props_->get_gobject (), 
					      status_enum_spec_, 
					      "status",
					      "Online Status");
    custom_status_spec_ = 
      sip_instance_->custom_props_->make_string_property ("status-note", 
							  "Custom status note",
							  "",
							  (GParamFlags) G_PARAM_READWRITE,
							  PJPresence::set_note,
							  PJPresence::get_note,
							  this);
    
    sip_instance_->install_property_by_pspec (sip_instance_->custom_props_->get_gobject (), 
					      custom_status_spec_, 
					      "status-note",
					      "Custom status note");
    

  }

  PJPresence::~PJPresence ()
  {
    unregister_account ();
  }

  gboolean
  PJPresence::register_account_wrapped (gchar *user, 
					gchar *domain, 
					gchar *password, 
					void *user_data)
  {
    PJPresence *context = static_cast<PJPresence *> (user_data);
    if (nullptr == user || nullptr == domain || nullptr == password)
      {
	g_warning ("register sip account received nullptr user or domain or password");
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

    
    //char ip_addr[32];
    /* Get local IP address for the default IP address */
    {
      const pj_str_t *hostname;
      pj_sockaddr_in tmp_addr;
      char *addr;
      
      hostname = pj_gethostname();
      pj_sockaddr_in_init(&tmp_addr, hostname, 0);
      addr = pj_inet_ntoa(tmp_addr.sin_addr);
      //pj_ansi_strcpy(ip_addr, addr);
    
      sip_local_user_ = std::string ("sip:") + sip_user + addr + std::to_string (sip_instance_->sip_port_);
      // sprintf (sip_local_user_,
      // 	       "sip:%s@%s:%u",
      // 	       sip_user.c_str (),
      // 	       addr,
      // 	       sip_instance_->sip_port_);
    }

    add_buddy ("sip:1000@10.10.30.179");
    add_buddy ("sip:1001@10.10.30.179");
    add_buddy ("sip:1002@10.10.30.179");
    add_buddy ("sip:1003@10.10.30.179");
    add_buddy ("sip:1004@10.10.30.179");
    add_buddy ("sip:1005@10.10.30.179");
    add_buddy ("sip:1006@10.10.30.179");
    add_buddy ("sip:1007@10.10.30.179");
    add_buddy ("sip:1008@10.10.30.179");
    add_buddy ("sip:1009@10.10.30.179");
    add_buddy ("sip:1010@10.10.30.179");
    add_buddy ("sip:1011@10.10.30.179");
    add_buddy ("sip:1012@10.10.30.179");
    add_buddy ("sip:1013@10.10.30.179");
    add_buddy ("sip:1014@10.10.30.179");
    add_buddy ("sip:1015@10.10.30.179");
    add_buddy ("sip:1016@10.10.30.179");
    add_buddy ("sip:1017@10.10.30.179");
    add_buddy ("sip:1018@10.10.30.179");
    add_buddy ("sip:1019@10.10.30.179");
  }


  gboolean
  PJPresence::unregister_account_wrapped (gpointer /*unused*/, void *user_data)
  {
    PJPresence *context = static_cast<PJPresence *> (user_data);
    context->sip_instance_->run_command_sync (std::bind (&PJPresence::unregister_account, 
							 context));
    if (-1 != context->account_id_)
      return FALSE;
    return TRUE;
  }

  bool
  PJPresence::unregister_account ()
  {
    std::unique_lock<std::mutex> lock (registration_mutex_);
    if (-1 == account_id_)
      return false;
    change_online_status (PJPresence::OFFLINE);
    if (PJ_SUCCESS != pjsua_acc_del (account_id_))
      {
	g_warning ("error when unregistering account");
	return false;
      }
    account_id_ = -1;
    
    sip_local_user_.clear ();
    return true;
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
    if (nullptr == context)
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

  void 
  PJPresence::set_status (const gint value, void *user_data)
  {
    PJPresence *context = static_cast <PJPresence *> (user_data);
    if (value < 0  || value >= OPT_MAX ) 
      {
	g_warning ("invalide online status code");
	return;
      }
    if (-1 == context->account_id_)
      {
	g_warning ("cannot set online status when not registered");
	return;
      }
    
    context->status_ = value;
    context->sip_instance_->run_command_sync (std::bind (&PJPresence::change_online_status, 
							 context,
							 context->status_));
    GObjectWrapper::notify_property_changed (context->sip_instance_->gobject_->get_gobject (), 
					     context->status_enum_spec_);

  }
  
  gint 
  PJPresence::get_status (void *user_data)
  {
    PJPresence *context = static_cast <PJPresence *> (user_data);
    return context->status_;
  }

  void 
  PJPresence::change_online_status (gint status) 
  {
    if (-1 == account_id_)
      return;
    pj_bool_t online_status = PJ_TRUE;
    pjrpid_element elem;
    pj_bzero(&elem, sizeof(elem));
    elem.type = PJRPID_ELEMENT_TYPE_PERSON;
    bool has_custom_status = true;
    char *tmp = nullptr; 
    On_scope_exit {if (nullptr != tmp) g_free (tmp);};
    if (custom_status_.empty () || 0 == custom_status_.compare (""))
      has_custom_status = false;
    else
      {
	tmp = g_strdup (custom_status_.c_str ());
	elem.note = pj_str(tmp);
      }
    
    switch (status) {
    case AVAILABLE:
      break;
    case BUSY:
      elem.activity = PJRPID_ACTIVITY_BUSY;
      if (!has_custom_status) pj_cstr(&elem.note, "Busy");
      break;
    case OTP:
      elem.activity = PJRPID_ACTIVITY_BUSY;
      if (!has_custom_status) pj_cstr(&elem.note, "On the phone");
      break;
    case IDLE:
      //elem.activity = PJRPID_ACTIVITY_UNKNOWN;
      elem.activity = PJRPID_ACTIVITY_AWAY;
      if (!has_custom_status) pj_cstr(&elem.note, "Idle");
      break;
    case AWAY:
      elem.activity = PJRPID_ACTIVITY_AWAY;
      if (!has_custom_status) pj_cstr(&elem.note, "Away");
      break;
    case BRB:
      //elem.activity = PJRPID_ACTIVITY_UNKNOWN;
      elem.activity = PJRPID_ACTIVITY_AWAY;
      if (!has_custom_status) pj_cstr(&elem.note, "Be right back");
      break;
    case OFFLINE:
      online_status = PJ_FALSE;
      break;
    }

    pjsua_acc_set_online_status2(account_id_, online_status, &elem);
  }

  void 
  PJPresence::set_note (const gchar *custom_status, void *user_data)
  {
    PJPresence *context = static_cast <PJPresence *> (user_data);
    if (0 == context->custom_status_.compare (custom_status))
      return;
    context->custom_status_ = custom_status;
    context->sip_instance_->run_command_sync (std::bind (&PJPresence::change_online_status, 
							 context,
							 context->status_));

    context->sip_instance_->custom_props_->notify_property_changed (context->custom_status_spec_);
  }

  const gchar *
  PJPresence::get_note (void *user_data)
  {
    PJPresence *context = static_cast <PJPresence *> (user_data);
    return context->custom_status_.c_str ();
  }

  /*
   * Handler registration status has changed.
   */
  void 
  PJPresence::on_reg_state(pjsua_acc_id acc_id)
  {
    PJ_UNUSED_ARG(acc_id);
    printf ("%s\n", __FUNCTION__);
    // Log already written.
  }
  
  /*
   * Handler for incoming presence subscription request
   */
  void 
  PJPresence::on_incoming_subscribe(pjsua_acc_id acc_id,
				    pjsua_srv_pres *srv_pres,
				    pjsua_buddy_id buddy_id,
				    const pj_str_t *from,
				    pjsip_rx_data *rdata,
				    pjsip_status_code *code,
				    pj_str_t *reason,
				    pjsua_msg_data *msg_data)
  {
    printf ("%s\n", __FUNCTION__);
    /* Just accept the request (the default behavior) */
    PJ_UNUSED_ARG(acc_id);
    PJ_UNUSED_ARG(srv_pres);
    PJ_UNUSED_ARG(buddy_id);
    PJ_UNUSED_ARG(from);
    PJ_UNUSED_ARG(rdata);
    PJ_UNUSED_ARG(code);
    PJ_UNUSED_ARG(reason);
    PJ_UNUSED_ARG(msg_data);
  }
  
  /*
   * Subscription state has changed.
   */
  void 
  PJPresence::on_buddy_evsub_state(pjsua_buddy_id buddy_id,
				   pjsip_evsub *sub,
				   pjsip_event *event)
  {
    printf ("%s\n", __FUNCTION__);
    char event_info[80];
    
    PJ_UNUSED_ARG(sub);
    
    event_info[0] = '\0';
    
    if (event->type == PJSIP_EVENT_TSX_STATE &&
	event->body.tsx_state.type == PJSIP_EVENT_RX_MSG)
      {
	pjsip_rx_data *rdata = event->body.tsx_state.src.rdata;
	snprintf(event_info, sizeof(event_info),
		 " (RX %s)",
		 pjsip_rx_data_get_info(rdata));
      }
    
    printf ("Buddy %d: subscription state: %s (event: %s%s)",
	    buddy_id, pjsip_evsub_get_state_name(sub),
	    pjsip_event_str(event->type),
	    event_info);
    
  }
 
}
