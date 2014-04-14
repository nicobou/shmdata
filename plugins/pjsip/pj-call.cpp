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
    /* Ignore strandled ACKs (must not send respone */
    if (rdata->msg_info.msg->line.req.method.id == PJSIP_ACK_METHOD)
      return PJ_FALSE;
    
    /* Respond (statelessly) any non-INVITE requests with 500  */
    if (rdata->msg_info.msg->line.req.method.id != PJSIP_INVITE_METHOD) {
      //FIXME 
      //pj_str_t reason = pj_str("Unsupported Operation");
      // pjsip_endpt_respond_stateless (sip_endpt_, rdata, 
      // 				     500, &reason,
      // 				     NULL, NULL);
      return PJ_TRUE;
    }
    
    /* Handle incoming INVITE */
    g_print ("TODO process incomming call\n");//process_incoming_call(rdata);
    
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
  
}
