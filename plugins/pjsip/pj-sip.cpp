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

#define SIP_DOMAIN	"10.10.30.115"
#define SIP_USER	"1000"
#define SIP_PASSWD	"1234"


namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PJSIP,
				       "SIP (Session Initiation Protocol)",
				       "network", 
				       "Manages user sessions",
				       "LGPL",
				       "sip",				
				       "Nicolas Bouillot");
  PJSIP::PJSIP ():
    pj_init_status_ (pj_init()),
    thread_handler_desc_ (),
    pj_thread_ref_ (NULL),
    sip_thread_ (std::thread (&PJSIP::sip_handling_thread, this))
    //sip_init_shutdown_thread_ (std::thread (&PJSIP::sip_init_shutdown_thread, this))
  {
    if (PJ_SUCCESS != pj_init_status_)
      g_warning ("pj_init () did not work");
    if (sip_thread_.joinable ())
      sip_thread_.join ();
    pj_shutdown();
  }

  // void 
  // PJSIP::sip_init_shutdown_thread ()
  // {
  //   if (PJ_SUCCESS != pj_init ())
  //     g_warning ("pj_init () did not work");
  //   std::thread sip_thread_ = std::thread (&PJSIP::sip_handling_thread, this);
  //   if (sip_thread_.joinable ())
  //     sip_thread_.join ();
  //   pj_shutdown();
  // }

  void 
  PJSIP::sip_handling_thread ()
  {
    // Register the thread, after pj_init() is called
     pj_thread_register("hehe",//Quiddity::get_name ().c_str (),
      		       thread_handler_desc_,
      		       &pj_thread_ref_);

     pjsua_acc_id acc_id;
     pj_status_t status;

     /* Create pjsua first! */
     status = pjsua_create();
     if (status != PJ_SUCCESS) 
       {
     	g_warning ("Error in pjsua_create()");
     	return;
       }

     //pjsua_pool_create();
   
     // /* If argument is specified, it's got to be a valid SIP URL */
     // if (argc > 1) {
     // 	status = pjsua_verify_url(argv[1]);
     // 	if (status != PJ_SUCCESS) error_exit("Invalid URL in argv", status);
     // }
    
     /* Init pjsua */
     {
       pjsua_config cfg;
       pjsua_logging_config log_cfg;
      
       pjsua_config_default(&cfg);
       // cfg.cb.on_incoming_call = &on_incoming_call;
       // cfg.cb.on_call_media_state = &on_call_media_state;
       // cfg.cb.on_call_state = &on_call_state;
      
       pjsua_logging_config_default(&log_cfg);
       log_cfg.console_level = 4;
      
       status = pjsua_init(&cfg, &log_cfg, NULL);
       if (status != PJ_SUCCESS) 
     	{
     	  g_warning ("Error in pjsua_init()");
     	  return;
     	}
     }

     /* Add UDP transport. */
     {
       pjsua_transport_config cfg;
	
       pjsua_transport_config_default(&cfg);
       cfg.port = 5060;
       status = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, NULL);
       if (status != PJ_SUCCESS) 
     	{
     	  g_warning ("Error creating transport");
     	  return;
     	}
	
       /* Initialization is done, now start pjsua */
       status = pjsua_start();
       if (status != PJ_SUCCESS) 
     	{
     	  g_warning ("Error starting pjsua");
     	  return;
     	}	    
       /* Register to SIP server by creating SIP account. */
       {
     	pjsua_acc_config cfg;
	      
     	pjsua_acc_config_default (&cfg);
     	cfg.id = pj_str (g_strdup ("sip:" SIP_USER "@" SIP_DOMAIN)); //FIXME free them  
     	cfg.reg_uri = pj_str (g_strdup ("sip:" SIP_DOMAIN));
     	cfg.cred_count = 1;
     	cfg.cred_info[0].realm = pj_str(g_strdup (SIP_DOMAIN));
     	cfg.cred_info[0].scheme = pj_str(g_strdup ("digest"));
     	cfg.cred_info[0].username = pj_str(g_strdup (SIP_USER));
     	cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
     	cfg.cred_info[0].data = pj_str(g_strdup (SIP_PASSWD));
	      
     	status = pjsua_acc_add(&cfg, PJ_TRUE, &acc_id);
     	if (status != PJ_SUCCESS) 
     	  {
     	    g_warning ("Error adding account");
     	    return;
     	  }
       }
	    
       // /* If URL is specified, make call to the URL. */
       // if (argc > 1) {
       //   pj_str_t uri = pj_str(argv[1]);
       //   status = pjsua_call_make_call(acc_id, &uri, 0, NULL, NULL, NULL);
       //   if (status != PJ_SUCCESS) error_exit("Error making call", status);
       // }
	    
       // /* Wait until user press "q" to quit. */
       // for (;;) {
       // 	char option[10];

       // 	puts("Press 'h' to hangup all calls, 'q' to quit");
       // 	if (fgets(option, sizeof(option), stdin) == NULL) {
       // 	    puts("EOF while reading stdin, will quit now..");
       // 	    break;
       // 	}

       // 	if (option[0] == 'q')
       // 	    break;

       // 	if (option[0] == 'h')
       // 	    pjsua_call_hangup_all();
     }
    
    pjsua_destroy();
  }

  PJSIP::~PJSIP ()
  {
     if (sip_thread_.joinable ())
       sip_thread_.join ();
    // if (sip_init_shutdown_thread_.joinable ())
    //   sip_init_shutdown_thread_.join ();
  }
  
  bool
  PJSIP::init ()
  {
    init_startable (this);
    g_debug ("hello from plugin");
    return true;
  }
    
  
  bool
  PJSIP::start ()
  {
    g_debug ("start from my plugin");
    return true;
  }

  bool
  PJSIP::stop ()
  {
    g_debug ("stop from my plugin");
    return true;
  }
}
