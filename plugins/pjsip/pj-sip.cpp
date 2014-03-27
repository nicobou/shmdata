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
#include <unistd.h>  //sleep

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
    thread_handler_desc_ (),
    pj_thread_ref_ (NULL),
    sip_thread_ ()
  {}

  PJSIP::~PJSIP ()
  {
    if (sip_thread_.joinable ())
      sip_thread_.join ();
  }

  bool
  PJSIP::init ()
  {
    init_startable (this);
    g_debug ("hello from pjsip plugin");
    sip_thread_ = std::thread (&PJSIP::sip_handling_thread, this);
    return true;
  }

  void 
  PJSIP::on_buddy_state(pjsua_buddy_id buddy_id)
  {
    pjsua_buddy_info info;
    //pjsua_buddy_update_pres (buddy_id);
    pjsua_buddy_get_info(buddy_id, &info);
    
    g_print ("!!!!!!!!!!!!!1 %.*s status is %.*s, subscription state is %s "
	     "(last termination reason code=%d %.*s)\n",
	     (int)info.uri.slen,
	     info.uri.ptr,
	     (int)info.status_text.slen,
	     info.status_text.ptr,
	     info.sub_state_name,
	     info.sub_term_code,
	     (int)info.sub_term_reason.slen,
	     info.sub_term_reason.ptr);
}

  void 
  PJSIP::sip_handling_thread ()
  {
    pj_init ();
    // Register the thread, after pj_init() is called
     pj_thread_register(Quiddity::get_name ().c_str (),
      		       thread_handler_desc_,
      		       &pj_thread_ref_);

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
       cfg.cb.on_buddy_state = &on_buddy_state;
       // cfg.cb.on_incoming_call = &on_incoming_call;
       // cfg.cb.on_call_media_state = &on_call_media_state;
       // cfg.cb.on_call_state = &on_call_state;
      
       pjsua_logging_config_default(&log_cfg);
       log_cfg.console_level = 1;//4;
      
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
       cfg.port = 5070;
       status = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, NULL);
       if (status != PJ_SUCCESS) 
	 {
	   g_warning ("Error creating UDP transport");
	   return;
	 }
     }	
     
      /* Add TCP transport. */
      {
        pjsua_transport_config cfg;
	
        pjsua_transport_config_default(&cfg);
        cfg.port = 5070;
        status = pjsua_transport_create(PJSIP_TRANSPORT_TCP, &cfg, NULL);
        if (status != PJ_SUCCESS) 
      	{
      	  g_warning ("Error creating TCP transport");
      	  return;
      	}
      }	

     /* Initialization is done, now start pjsua */
     status = pjsua_start();
     if (status != PJ_SUCCESS) 
       {
	 g_warning ("Error starting pjsua");
	 return;
       }	    

     /* Register to SIP server by creating SIP account. */
     pjsua_acc_id acc_id;
     {
       pjsua_acc_config cfg;
       
       pjsua_acc_config_default (&cfg);
       cfg.id = pj_str ("sip:" SIP_USER "@" SIP_DOMAIN); 
       cfg.reg_uri = pj_str ("sip:" SIP_DOMAIN);
       cfg.cred_count = 1;
       cfg.cred_info[0].realm = pj_str(SIP_DOMAIN);
       cfg.cred_info[0].scheme = pj_str("digest");
       cfg.cred_info[0].username = pj_str(SIP_USER);
       cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
       cfg.cred_info[0].data = pj_str(SIP_PASSWD);
       cfg.publish_enabled = PJ_TRUE; 
       status = pjsua_acc_add(&cfg, PJ_TRUE, &acc_id);
       if (status != PJ_SUCCESS) 
	 {
	   g_warning ("Error adding account");
	   return;
	 }
     }
     
     {
       pjsua_buddy_config buddy_cfg;
       pjsua_buddy_id buddy_id;
       pj_status_t status = PJ_SUCCESS;
       
       if (pjsua_verify_url("sip:1001@10.10.30.115") != PJ_SUCCESS) {
	 g_debug ("Invalid URI");
       } else {
	 pj_bzero(&buddy_cfg, sizeof(pjsua_buddy_config));
	 
	 buddy_cfg.uri = pj_str("sip:1001@10.10.30.115");
	 buddy_cfg.subscribe = PJ_TRUE;
	 
	 status = pjsua_buddy_add(&buddy_cfg, &buddy_id);
	 if (status == PJ_SUCCESS) {
	   g_debug ("New buddy");
	 }

	 //	 pjsua_buddy_subscribe_pres
       }
     }

     {
       pjsua_buddy_config buddy_cfg;
       pjsua_buddy_id buddy_id;
       pj_status_t status = PJ_SUCCESS;
       
       if (pjsua_verify_url("sip:1002@10.10.30.115") != PJ_SUCCESS) {
	 g_debug ("Invalid URI");
       } else {
	 pj_bzero(&buddy_cfg, sizeof(pjsua_buddy_config));
	 
	 buddy_cfg.uri = pj_str("sip:1002@10.10.30.115");
	 buddy_cfg.subscribe = PJ_TRUE;
	 
	 status = pjsua_buddy_add(&buddy_cfg, &buddy_id);
	 if (status == PJ_SUCCESS) {
	   g_debug ("New buddy");
	 }

	 //	 pjsua_buddy_subscribe_pres
       }
     }
     
     {
       pjrpid_element elem;
       pj_bool_t online_status = PJ_TRUE;
       pj_bzero(&elem, sizeof(elem));
       elem.type = PJRPID_ELEMENT_TYPE_PERSON;
       
       // switch (choice) {
       // case AVAILABLE:
       // 	break;
       // case BUSY:
       // 	elem.activity = PJRPID_ACTIVITY_BUSY;
       // 	elem.note = pj_str("Busy");
       // 	break;
       // case OTP:
       // 	elem.activity = PJRPID_ACTIVITY_BUSY;
       // 	elem.note = pj_str("On the phone");
       // 	break;
       // case IDLE:
       // 	elem.activity = PJRPID_ACTIVITY_UNKNOWN;
       // 	elem.note = pj_str("Idle");
       // 	break;
       // case AWAY:
       elem.activity = PJRPID_ACTIVITY_AWAY;
       elem.note = pj_str("Away");
       // 	break;
       // case BRB:
       // elem.activity = PJRPID_ACTIVITY_UNKNOWN;
       // elem.note = pj_str("Be right back SWITCHER");
       // 	break;
       // case OFFLINE:
       // 	online_status = PJ_FALSE;
       // 	break;
       // }
       pjsua_acc_set_online_status2(acc_id, online_status, &elem);
     }
       
     usleep (2000000);

     {
       pjrpid_element elem;
       pj_bool_t online_status = PJ_TRUE;
       pj_bzero(&elem, sizeof(elem));
       elem.type = PJRPID_ELEMENT_TYPE_PERSON;
       
       elem.activity = PJRPID_ACTIVITY_BUSY;
       elem.note = pj_str("Busy");
       pjsua_acc_set_online_status2(acc_id, online_status, &elem);
     }
       
     usleep (2000000);
     
     pjsua_destroy();
     pj_shutdown ();
  }

    
  
  bool
  PJSIP::start ()
  {
    g_debug ("start from pjsip plugin");
    return true;
  }

  bool
  PJSIP::stop ()
  {
    g_debug ("stop from pjsip plugin");
    return true;
  }
}
