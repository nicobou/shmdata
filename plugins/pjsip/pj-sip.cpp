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
    worker_thread_ref_ (NULL)
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
						  "SIP Domain", //first arg long name
						  "domain", //fisrt arg name
						  "string", //first arg description
						  "SIP password", //first arg long name
						  "password", //fisrt arg name
						  "string", //first arg description
						  NULL),
  		    (Method::method_ptr) &register_account_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, 
						       G_TYPE_STRING, 
						       G_TYPE_STRING, 
						       NULL),
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

    pj_log_set_level(5);

    // Register the thread, after pj_init() is called
    pj_thread_register(Quiddity::get_name ().c_str (),
		       thread_handler_desc_,
		       &pj_thread_ref_);
    
    /* init PJLIB-UTIL: */
    status = pjlib_util_init();
    PJ_ASSERT_RETURN(status == PJ_SUCCESS, status);
    
    /* Must create a pool factory before we can allocate any memory. */
    pj_caching_pool_init(&cp_, &pj_pool_factory_default_policy, 0);
    
    /* Create application pool for misc. */
    pool_ = pj_pool_create(&cp_.factory, "switcher_sip", 1000, 1000, NULL);
    
    /* Create the endpoint: */
    status = pjsip_endpt_create(&cp_.factory, 
				pj_gethostname()->ptr, 
				&sip_endpt_);
  if (status != PJ_SUCCESS) {
    g_warning ("Unable to create sip end point");
    return false;
  }
    
  /* Add UDP transport. */
    {
	pj_sockaddr_in addr;
	pjsip_transport *tp;

	pj_bzero(&addr, sizeof(addr));
	addr.sin_family = pj_AF_INET();
	addr.sin_addr.s_addr = 0;
	addr.sin_port = pj_htons((pj_uint16_t)5072);

	// pjsip_host_port addrname;
	// if (app.local_addr.slen) {

	//     addrname.host = app.local_addr;
	//     addrname.port = app.sip_port;

	//     status = pj_sockaddr_in_init(&addr, &app.local_addr, 
	// 				 (pj_uint16_t)app.sip_port);
	//     if (status != PJ_SUCCESS) {
	// 	app_perror(THIS_FILE, "Unable to resolve IP interface", status);
	// 	return status;
	//     }
	// }

	status = pjsip_udp_transport_start (sip_endpt_, &addr, 
					    //(app.local_addr.slen ? &addrname:NULL),
					    NULL,
					    1, &tp);
	if (status != PJ_SUCCESS) {
	    g_warning ("Unable to start UDP transport");
	    return false;
	}
  }//end add DUP transport

  /* 
   * Init transaction layer.
   * This will create/initialize transaction hash tables etc.
   */
  status = pjsip_tsx_layer_init_module(sip_endpt_);
  if (status != PJ_SUCCESS) {
    g_warning ("Unable to start transaction layer");
    return false;
  }
  
  /*  Initialize UA layer. */
  status = pjsip_ua_init_module(sip_endpt_, NULL );
  if (status != PJ_SUCCESS) {
    g_warning ("Unable to Initialize UA layer ");
    return false;
  }
  
  /* Initialize 100rel support */
  status = pjsip_100rel_init_module(sip_endpt_);
  if (status != PJ_SUCCESS) {
    g_warning ("Unable to Init 100rel support");
    return false;
  }
  
  sip_calls_ = new PJCall (this);
  
  sip_work_ = true;
  sip_worker_ = std::thread (&PJSIP::sip_worker_thread, this);

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
    //std::unique_lock<std::mutex> lock (registration_mutex_);
    //registration_cond_.wait (lock);
    add_buddy ("sip:1000@10.10.30.115");
    add_buddy ("sip:1001@10.10.30.115");
    add_buddy ("sip:1002@10.10.30.115");
    add_buddy ("sip:1003@10.10.30.115");
    add_buddy ("sip:1004@10.10.30.115");
    add_buddy ("sip:1005@10.10.30.115");
    add_buddy ("sip:1006@10.10.30.115");
    add_buddy ("sip:1007@10.10.30.115");
    add_buddy ("sip:1008@10.10.30.115");
    add_buddy ("sip:1009@10.10.30.115");
    add_buddy ("sip:1010@10.10.30.115");
    add_buddy ("sip:1011@10.10.30.115");
    add_buddy ("sip:1012@10.10.30.115");
    add_buddy ("sip:1013@10.10.30.115");
    add_buddy ("sip:1014@10.10.30.115");
    add_buddy ("sip:1015@10.10.30.115");
    add_buddy ("sip:1016@10.10.30.115");
    add_buddy ("sip:1017@10.10.30.115");
    add_buddy ("sip:1018@10.10.30.115");
    add_buddy ("sip:1019@10.10.30.115");
  }

  void
  PJSIP::add_buddy (const std::string &sip_user)
  {
  }
  
  void
  PJSIP::exit_cmd ()
  {
    continue_ = false;
  }
  


   void 
   PJSIP::set_port (const gint value, void *user_data)
   {
     PJSIP *context = static_cast <PJSIP *> (user_data);
     context->sip_port_ = value;
     GObjectWrapper::notify_property_changed (context->gobject_->get_gobject (),
					      context->sip_port_spec_);
   }
   
  gint 
  PJSIP::get_port (void *user_data)
  {
    PJSIP *context = static_cast <PJSIP *> (user_data);
    return context->sip_port_;
  }

}
