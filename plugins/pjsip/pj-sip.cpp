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

#include "pj-sip.h"

namespace switcher
{

  SWITCHER_DECLARE_PLUGIN (PJSIP);
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PJSIP,
				       "SIP (Session Initiation Protocol)",
				       "network", 
				       "Manages user sessions",
				       "LGPL",
				       "sip",				
				       "Nicolas Bouillot");

  //according to pjsip documentation:
  //Application should only instantiate one SIP endpoint instance for every process. 
  pjsip_endpoint *PJSIP::sip_endpt_ = NULL;

  PJSIP::PJSIP ():
    custom_props_ (std::make_shared<CustomPropertyHelper> ()),
    sip_port_ (5060),
    sip_port_spec_ (NULL),
    thread_handler_desc_ (),
    pj_thread_ref_ (NULL),
    transport_id_ (NULL),//udp_transport_ (NULL),
    sip_thread_ (),
    pj_init_mutex_ (),
    pj_init_cond_ (),
    pj_sip_inited_ (false),
    work_mutex_ (),
    work_cond_ (),
    done_mutex_ (),
    done_cond_ (),
    registration_mutex_ (),
    registration_cond_ (),
    continue_ (true),
    command_ (),
    cp_ (),
    pool_ (NULL),
    sip_calls_ (NULL),
    sip_worker_ (),
    sip_work_ (true),
    worker_handler_desc_ (),
    worker_thread_ref_ (NULL),
    account_id_ (-1)
  {}

  PJSIP::~PJSIP ()
  {
    if (!pj_sip_inited_)
      return;

    run_command_sync (std::bind (&PJSIP::exit_cmd, this));
   
    if (sip_thread_.joinable ())
      sip_thread_.join ();
    if (sip_worker_.joinable ())
      {
	sip_work_ = false;
	sip_worker_.join ();
      }
  }

  
  void 
  PJSIP::run_command_sync (std::function<void()> command)
  {
    {
      std::unique_lock<std::mutex> lock (work_mutex_);
      command_ = command;
    }
    std::unique_lock<std::mutex> lock_done (done_mutex_);
    work_cond_.notify_one();
    done_cond_.wait (lock_done);
  }
  
  bool
  PJSIP::init ()
  {
    if (NULL != sip_endpt_) 
      {
	g_warning ("a pjsip_endpoint already exists, cannot create more");
	return false;
      }

    std::unique_lock <std::mutex> lock (pj_init_mutex_);
    sip_thread_ = std::thread (&PJSIP::sip_handling_thread, this);
    pj_init_cond_.wait (lock);

    if (!pj_sip_inited_)
      return false;

    install_method ("Register SIP Account", //long name
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

    install_method ("Call a contact", //long name
		    "call", //name
		    "invite a contact for a call", //description
		    "the call has been initiated or not", //return description
		    Method::make_arg_description ("SIP url", //first arg long name
						  "url", //fisrt arg name
						  "string", //first arg description
						  NULL),
  		    (Method::method_ptr) &call_sip_url, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);

    sip_port_spec_ =
      custom_props_->make_int_property ("port", 
					"SIP port used when registering",
					0,
					65535,
					sip_port_,
					(GParamFlags) G_PARAM_READWRITE,
					set_port,
					get_port,
					this);

    install_property_by_pspec (custom_props_->get_gobject (), 
			       sip_port_spec_, 
			       "port",
			       "SIP port used when registering");

    return true;
  }

  bool
  PJSIP::pj_sip_init ()
  {
    pj_status_t status = pj_init();
    if (status != PJ_SUCCESS)
      return false;

    pj_log_set_level(6);

    // Register the thread, after pj_init() is called
    pj_thread_register(Quiddity::get_name ().c_str (),
		       thread_handler_desc_,
		       &pj_thread_ref_);
    

    status = pjsua_create();
    if (status != PJ_SUCCESS) 
      {
	g_warning ("Error in pjsua_create()");
	return false;
      }
    /* Init pjsua */
      {
	pjsua_config cfg;
	pjsua_logging_config log_cfg;
	
	pjsua_config_default(&cfg);
	cfg.cb.on_buddy_state = &on_buddy_state;
	cfg.cb.on_reg_state2 = &on_registration_state;
	//cfg.cb.on_create_media_transport = &on_create_media_transport;---
 
	//see pjsip-apps/src/pjsua/pjsua_app.c
	// cfg.cb.on_call_state = &on_call_state;
	//cfg.cb.on_call_media_state = &on_call_media_state;---
	//cfg.cb.on_incoming_call = &on_incoming_call;---
	// cfg.cb.on_call_tsx_state = &on_call_tsx_state;
	// cfg.cb.on_dtmf_digit = &call_on_dtmf_callback;
	// cfg.cb.on_call_redirected = &call_on_redirected;
	// cfg.cb.on_reg_state = &on_reg_state;
	// cfg.cb.on_incoming_subscribe = &on_incoming_subscribe;
	// cfg.cb.on_buddy_evsub_state = &on_buddy_evsub_state;
	// cfg.cb.on_pager = &on_pager;
	// cfg.cb.on_typing = &on_typing;
	// cfg.cb.on_call_transfer_status = &on_call_transfer_status;
	// cfg.cb.on_call_replaced = &on_call_replaced;
	// cfg.cb.on_nat_detect = &on_nat_detect;
	// cfg.cb.on_mwi_info = &on_mwi_info;
	// cfg.cb.on_transport_state = &on_transport_state;
	// cfg.cb.on_ice_transport_error = &on_ice_transport_error;
	// cfg.cb.on_snd_dev_operation = &on_snd_dev_operation;
	// cfg.cb.on_call_media_event = &on_call_media_event;

	pjsua_logging_config_default(&log_cfg);
	log_cfg.console_level = 1;
	
	status = pjsua_init(&cfg, &log_cfg, NULL);
	if (status != PJ_SUCCESS) 
	  {
	    g_warning ("Error in pjsua_init()");
	    return false;
	  }
	sip_endpt_ = pjsua_get_pjsip_endpt ();	
      }
    
      /* Must create a pool factory before we can allocate any memory. */
      pj_caching_pool_init(&cp_, &pj_pool_factory_default_policy, 0);
      
    /* Create application pool for misc. */
    pool_ = pj_pool_create(&cp_.factory, "switcher_sip", 1000, 1000, NULL);
    

    start_udp_transport ();
    
    sip_calls_ = new PJCall (this);
    
    sip_work_ = true;
    sip_worker_ = std::thread (&PJSIP::sip_worker_thread, this);
    
    /* Initialization is done, now start pjsua */
    status = pjsua_start();
    if (status != PJ_SUCCESS) 
      {
	g_warning ("Error starting pjsua");
	return false;
      }	    
    
  return true;    
  }
  
  void 
  PJSIP::sip_worker_thread ()
  {
    // Register the thread, after pj_init() is called
    
    pj_thread_register("sip_worker_thread",
		       worker_handler_desc_,
		       &worker_thread_ref_);
    
    while (sip_work_) 
      {
	pj_time_val timeout = {0, 10};
	pjsip_endpt_handle_events(sip_endpt_, 
				  &timeout);
      }
  }

  void 
  PJSIP::sip_handling_thread ()
  {
    {//init pj sip
      std::unique_lock <std::mutex> lock (pj_init_mutex_);
      pj_sip_inited_ = pj_sip_init ();
      pj_init_cond_.notify_all ();
    }

    while (continue_)
      {
	  std::unique_lock<std::mutex> lock_work (work_mutex_);
	  work_cond_.wait (lock_work);
	  //do_something
	  {
	    std::unique_lock<std::mutex> lock_done (done_mutex_);
	    command_ ();
	  }
	  done_cond_.notify_one ();
      }

    /* Shutting down... */
    if (NULL != sip_calls_)
      delete (sip_calls_);

    //destroy_media();
    
    if (NULL != sip_endpt_) 
      {
	pjsip_endpt_destroy(sip_endpt_);
	sip_endpt_ = NULL;
      }
    
    if (NULL != pool_) {
      pj_pool_release(pool_);
      pool_ = NULL;
      pj_caching_pool_destroy(&cp_);
    }
    pj_shutdown ();
    //g_print ("PJ shutdowned\n");
  }
  
  gboolean
  PJSIP::register_account_wrapped (gchar *user, gchar *domain, gchar *password, void *user_data)
  {
    PJSIP *context = static_cast<PJSIP *> (user_data);
    if (NULL == user || NULL == domain || NULL == password)
      {
	g_warning ("register sip account received NULL user or domain or password");
	return FALSE;
      }

    context->run_command_sync (std::bind (&PJSIP::register_account, 
					  context, 
					  std::string (user),
					  std::string (domain),
					  std::string (password)));

    return TRUE;
  }

  void 
  PJSIP::register_account (const std::string &sip_user, 
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
  PJSIP::add_buddy (const std::string &sip_user)
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
  PJSIP::exit_cmd ()
  {
    continue_ = false;
  }
  
  void
  PJSIP::start_udp_transport ()
  {
    
    if (NULL != transport_id_)
      pjsua_transport_close (*transport_id_, PJ_FALSE);
    
    pjsua_transport_config cfg;
    pjsua_transport_config_default(&cfg);
    cfg.port = sip_port_;
    pj_status_t status = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, NULL);
    if (status != PJ_SUCCESS) 
      {
	g_warning ("Error creating UDP transport");
	return;
      }
    
  // /* Add UDP transport. */
  //   pj_sockaddr_in addr;
    
  //   pj_bzero(&addr, sizeof(addr));
  //   addr.sin_family = pj_AF_INET();
  //   addr.sin_addr.s_addr = 0;
  //   addr.sin_port = pj_htons((pj_uint16_t)sip_port_);
    
  //   if (NULL == udp_transport_)
  //     {
  // 	if (PJ_SUCCESS != pjsip_udp_transport_start (sip_endpt_, 
  // 						     &addr, 
  // 						     NULL,
  // 						     1, 
  // 						     &udp_transport_))
  // 	g_warning ("Unable to start UDP transport");
  //     }
  //   else
  //     {
  // 	if (PJ_SUCCESS != pjsip_udp_transport_restart (udp_transport_,
  // 						       PJSIP_UDP_TRANSPORT_DESTROY_SOCKET,
  // 						       PJ_INVALID_SOCKET,
  // 						       &addr,
  // 						       NULL))
  // 	g_warning ("Unable to restart UDP transport");
  //     }
    
  }
  
   void 
   PJSIP::set_port (const gint value, void *user_data)
   {
     PJSIP *context = static_cast <PJSIP *> (user_data);
     context->sip_port_ = value;
     context->run_command_sync (std::bind (&PJSIP::start_udp_transport, 
					   context));
     GObjectWrapper::notify_property_changed (context->gobject_->get_gobject (),
					      context->sip_port_spec_);
   }
   
  gint 
  PJSIP::get_port (void *user_data)
  {
    PJSIP *context = static_cast <PJSIP *> (user_data);
    return context->sip_port_;
  }

  gboolean
  PJSIP::call_sip_url (gchar *sip_url, void *user_data)
  {
    if (NULL == sip_url || NULL == user_data)
      {
	g_warning ("register sip account received NULL user or domain or password");
	return FALSE;
      }
    PJSIP *context = static_cast<PJSIP *> (user_data);

    context->run_command_sync (std::bind (&PJCall::make_call, 
					  context->sip_calls_, 
					  std::string (sip_url)));

    return TRUE;
  }

  void 
  PJSIP::on_registration_state (pjsua_acc_id acc_id, pjsua_reg_info *info)
  {
    PJSIP *context = static_cast<PJSIP *> (pjsua_acc_get_user_data (acc_id));
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
  PJSIP::on_buddy_state(pjsua_buddy_id buddy_id)
  {
    PJSIP *context = static_cast<PJSIP *> (pjsua_buddy_get_user_data (buddy_id));
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
     context->graft_tree (std::string (".presence." + buddy_url), tree);
  }


}
