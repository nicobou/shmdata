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

#include "pj-call.h"
#include "pj-sip.h"

namespace switcher
{

  pjsip_module PJCall::mod_siprtp_ =
    {
      NULL, NULL,			    /* prev, next.		*/
      { "mod-siprtpapp", 13 },	    /* Name.			*/
      -1,				    /* Id			*/
      PJSIP_MOD_PRIORITY_APPLICATION, /* Priority			*/
      NULL,			    /* load()			*/
      NULL,			    /* start()			*/
      NULL,			    /* stop()			*/
      NULL,			    /* unload()			*/
      &on_rx_request,		    /* on_rx_request()		*/
      NULL,			    /* on_rx_response()		*/
      NULL,			    /* on_tx_request.		*/
      NULL,			    /* on_tx_response()		*/
      NULL,			    /* on_tsx_state()		*/
    };
  

  PJCall::PJCall (PJSIP *sip_instance) :
    med_endpt_ (NULL)
  {
    pj_status_t status;
    /*  Init invite session module. */
    {
      pjsip_inv_callback inv_cb;
      
      /* Init the callback for INVITE session: */
      pj_bzero(&inv_cb, sizeof(inv_cb));
      inv_cb.on_state_changed = &call_on_state_changed;
      inv_cb.on_new_session = &call_on_forked;
      inv_cb.on_media_update = &call_on_media_update;
      
      /* Initialize invite session module:  */
      status = pjsip_inv_usage_init(sip_instance->sip_endpt_, &inv_cb);
      if (status != PJ_SUCCESS) 
	g_warning ("Init invite session module failed");
    }
    
    /* Register our module to receive incoming requests. */
    status = pjsip_endpt_register_module(sip_instance->sip_endpt_, &mod_siprtp_);
    if (status != PJ_SUCCESS) 
      g_warning ("Register mod_siprtp_ failed");

    // /* Init media */
    status = pjmedia_endpt_create(&sip_instance->cp_.factory, NULL, 1, &med_endpt_);
      if (status != PJ_SUCCESS) 
	g_warning ("Init media failed");

      
    // /* Init calls */
    // for (i=0; i<app.max_calls; ++i)
    //   app.call[i].index = i;
    g_print ("pj_call initialized\n");
  }

  PJCall::~PJCall ()
  {
    // unsigned i;
    // app.thread_quit = 1;
    // for (i=0; i<app.thread_count; ++i) {
    // 	if (app.sip_thread[i]) {
    // 	    pj_thread_join(app.sip_thread[i]);
    // 	    pj_thread_destroy(app.sip_thread[i]);
    // 	    app.sip_thread[i] = NULL;
    // 	}
    // }

    // unsigned i;
    // for (i=0; i<app.max_calls; ++i) {
    // 	unsigned j;
    // 	for (j=0; j<PJ_ARRAY_SIZE(app.call[0].media); ++j) {
    // 	    struct media_stream *m = &app.call[i].media[j];

    // 	    if (m->transport) {
    // 		pjmedia_transport_close(m->transport);
    // 		m->transport = NULL;
    // 	    }
    // 	}
    // }

    if (med_endpt_) {
      pjmedia_endpt_destroy(med_endpt_);
      med_endpt_ = NULL;
    }
    g_print ("pj_call destructed\n");
  }

  /* Callback to be called to handle incoming requests outside dialogs: */
  pj_bool_t 
  PJCall::on_rx_request (pjsip_rx_data *rdata)
  {
    g_print ("--> %s\n", __FUNCTION__);
    /* Ignore strandled ACKs (must not send respone */
    if (rdata->msg_info.msg->line.req.method.id == PJSIP_ACK_METHOD)
      return PJ_FALSE;
    
    /* Respond (statelessly) any non-INVITE requests with 500  */
    if (rdata->msg_info.msg->line.req.method.id != PJSIP_INVITE_METHOD) 
      {
      //FIXME 
      pj_str_t reason = pj_str("Unsupported Operation");
      pjsip_endpt_respond_stateless (rdata->tp_info.transport->endpt, 
				     rdata, 
       				     500, &reason,
       				     NULL, NULL);
      return PJ_TRUE;
    }
    
    /* Handle incoming INVITE */
    process_incoming_call (rdata);

    /* Done */
    return PJ_TRUE;

  }
  
  /* Callback to be called when invite session's state has changed: */
  void 
  PJCall::call_on_state_changed( pjsip_inv_session *inv, 
				 pjsip_event *e)
  {
    g_print ("--> %s\n", __FUNCTION__);
  }

  /* Callback to be called when dialog has forked: */
  void 
  PJCall::call_on_forked(pjsip_inv_session *inv, pjsip_event *e)
  {
    g_print ("--> %s\n", __FUNCTION__);
  }
  
  /* Callback to be called when SDP negotiation is done in the call: */
  void 
  PJCall::call_on_media_update( pjsip_inv_session *inv,
				pj_status_t status)
  {
    g_print ("--> %s\n", __FUNCTION__);
  }


/*
 * Receive incoming call
 */
  void 
  PJCall::process_incoming_call (pjsip_rx_data *rdata)
  {
    g_print ("TODO --> %s, name %.*s, short name %.*s, tag %.*s\n", 
	     __FUNCTION__,
	     (int)rdata->msg_info.from->name.slen,
	     rdata->msg_info.from->name.ptr,
	     (int)rdata->msg_info.from->sname.slen,
	     rdata->msg_info.from->sname.ptr,
	     (int)rdata->msg_info.from->tag.slen,
	     rdata->msg_info.from->tag.ptr);
    
    //unsigned i;
    unsigned options;
    //struct call *call;
    pjsip_dialog *dlg;
    //pjmedia_sdp_session *sdp;
    pjsip_tx_data *tdata;
    pj_status_t status;

    // /* Find free call slot */
    // for (i=0; i<app.max_calls; ++i) {
    // 	if (app.call[i].inv == NULL)
    // 	    break;
    // }

    // if (i == app.max_calls) {
    // 	const pj_str_t reason = pj_str("Too many calls");
    // 	pjsip_endpt_respond_stateless( app.sip_endpt, rdata, 
    // 				       500, &reason,
    // 				       NULL, NULL);
    // 	return;
    // }

    // call = &app.call[i];

    /* Verify that we can handle the request. */
    options = 0;
    status = pjsip_inv_verify_request(rdata, &options, NULL, NULL,
				      PJSIP::sip_endpt_, &tdata);
    if (status != PJ_SUCCESS) {
     	/*
     	 * No we can't handle the incoming INVITE request.
     	 */
     	if (tdata) {
     	    pjsip_response_addr res_addr;
	    
     	    pjsip_get_response_addr(tdata->pool, rdata, &res_addr);
     	    pjsip_endpt_send_response(PJSIP::sip_endpt_, &res_addr, tdata,
     		NULL, NULL);
	    
     	} else {
	    
     	    /* Respond with 500 (Internal Server Error) */
	  pjsip_endpt_respond_stateless(PJSIP::sip_endpt_, rdata, 500, NULL,
     		NULL, NULL);
     	}
	
     	return;
     }

    /* Create UAS dialog */
    status = pjsip_dlg_create_uas (pjsip_ua_instance(), rdata,
     				   NULL, //FIXME was &app.local_contact
				   &dlg);
    if (status != PJ_SUCCESS) {
      const pj_str_t reason = pj_str ("Unable to create dialog");
      pjsip_endpt_respond_stateless (PJSIP::sip_endpt_, rdata, 
				    500, &reason,
				    NULL, NULL);
      return;
    }
    
     /* Create SDP */
    //FIXME create according to the caller 
    //create_sdp (dlg->pool, call, &sdp);
    
    // /* Create UAS invite session */
    // status = pjsip_inv_create_uas (dlg, rdata, sdp, 0, &call->inv);
    // if (status != PJ_SUCCESS) {
    //   pjsip_dlg_create_response(dlg, rdata, 500, NULL, &tdata);
    //   pjsip_dlg_send_response(dlg, pjsip_rdata_get_tsx(rdata), tdata);
    //   return;
    // }
    
    // /* Attach call data to invite session */
    // call->inv->mod_data[mod_siprtp.id] = call;
    
    // /* Mark start of call */
    // pj_gettimeofday(&call->start_time);
    
    
    
    // /* Create 200 response .*/
    // status = pjsip_inv_initial_answer(call->inv, rdata, 200, 
    //  				      NULL, NULL, &tdata);
    // if (status != PJ_SUCCESS) {
    //   status = pjsip_inv_initial_answer(call->inv, rdata, 
    // 					PJSIP_SC_NOT_ACCEPTABLE,
    // 					NULL, NULL, &tdata);
    //   if (status == PJ_SUCCESS)
    // 	pjsip_inv_send_msg(call->inv, tdata); 
    //   else
    // 	pjsip_inv_terminate(call->inv, 500, PJ_FALSE);
    //   return;
    // }
    
    
    // /* Send the 200 response. */  
    // status = pjsip_inv_send_msg(call->inv, tdata); 
    // PJ_ASSERT_ON_FAIL(status == PJ_SUCCESS, return);
    

   /* Done */
  }

}
