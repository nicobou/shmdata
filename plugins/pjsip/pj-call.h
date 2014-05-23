/*
 * This file is part of switcher-myplugin.
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

#ifndef __SWITCHER_CALL_H__
#define __SWITCHER_CALL_H__

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

//pjsip
#include <pjsip.h>
#include <pjmedia.h>
#include <pjmedia-codec.h>
#include <pjsip_ua.h>
#include <pjsip_simple.h>
#include <pjlib-util.h>
#include <pjlib.h>

#include "pj-codec.h"

#define MAX_CALLS 1024

namespace switcher
{
  class PJSIP;

  class PJCall
  {
    friend PJCodec;
    
  public:
    PJCall () = delete;
    PJCall (PJSIP *sip_instance);
    ~PJCall ();
    PJCall (const PJCall &) = delete;
    PJCall &operator= (const PJCall &) = delete;

  private:
/* Codec descriptor: */
typedef struct codec
{
    unsigned	pt;
    char*	name;
    unsigned	clock_rate;
    unsigned	bit_rate;
    unsigned	ptime;
    char*	description;
} codec_t;


/* A bidirectional media stream created when the call is active. */
struct media_stream
{
    /* Static: */
    unsigned		 call_index;	    /* Call owner.		*/
    unsigned		 media_index;	    /* Media index in call.	*/
    pjmedia_transport   *transport;	    /* To send/recv RTP/RTCP	*/

    /* Active? */
    pj_bool_t		 active;	    /* Non-zero if is in call.	*/

    /* Current stream info: */
    pjmedia_stream_info	 si;		    /* Current stream info.	*/

    /* More info: */
    unsigned		 clock_rate;	    /* clock rate		*/
    unsigned		 samples_per_frame; /* samples per frame	*/
    unsigned		 bytes_per_frame;   /* frame size.		*/

    /* RTP session: */
    pjmedia_rtp_session	 out_sess;	    /* outgoing RTP session	*/
    pjmedia_rtp_session	 in_sess;	    /* incoming RTP session	*/

    /* RTCP stats: */
    pjmedia_rtcp_session rtcp;		    /* incoming RTCP session.	*/

    /* Thread: */
    pj_bool_t		 thread_quit_flag;  /* Stop media thread.	*/
    pj_thread_t		*thread;	    /* Media thread.		*/
};


/* This is a call structure that is created when the application starts
 * and only destroyed when the application quits.
 */
struct call
{
  unsigned		 index;
  pjsip_inv_session	*inv;
  unsigned		 media_count;
  struct media_stream	 media[1];
  pj_time_val		 start_time;
  pj_time_val		 response_time;
  pj_time_val		 connect_time;
  
  pj_timer_entry	 d_timer;	    /**< Disconnect timer.	*/
};


/* Application's global variables */
typedef struct app
{
    unsigned		 max_calls;
    unsigned		 call_gap;
    pj_bool_t		 call_report;
    unsigned		 uac_calls;
    unsigned		 duration;
    pj_bool_t		 auto_quit;
    unsigned		 thread_count;
    int			 sip_port;
    int			 rtp_start_port;
    pj_str_t		 local_addr;
    pj_str_t		 local_uri;
    pj_str_t		 local_contact;
    
    int			 app_log_level;
    int			 log_level;
    char		*log_filename;
    char		*report_filename;

    struct codec	 audio_codec;

    pj_str_t		 uri_to_call;

    pj_caching_pool	 cp;
    pj_pool_t		*pool;

  //pjsip_endpoint	*sip_endpt;
    pj_bool_t		 thread_quit;
    pj_thread_t		*sip_thread[1];

  //pjmedia_endpt	*med_endpt;
    struct call		 call[MAX_CALLS];
} app_t;

 typedef struct alt_codec
{
    pj_str_t	encoding_name;
    pj_uint8_t	payload_type;
    unsigned	clock_rate;
    unsigned	channel_cnt;
    unsigned	frm_ptime;
    unsigned	avg_bps;
    unsigned	max_bps;
 } alt_codec_t;



  private:
    static pjmedia_endpt *med_endpt_;
    static pjsip_module mod_siprtp_;
    static app_t app;
    static struct codec audio_codecs[];
    static pj_bool_t on_rx_request (pjsip_rx_data *rdata);
    static void call_on_state_changed (pjsip_inv_session *inv, 
				       pjsip_event *e);
    static void call_on_forked (pjsip_inv_session *inv, pjsip_event *e);
    static void call_on_media_update (pjsip_inv_session *inv,
				      pj_status_t status);
    static void process_incoming_call (pjsip_rx_data *rdata);
    void init_app ();
    static pj_status_t create_sdp( pj_pool_t *pool,
				   struct call *call,
				   pjmedia_sdp_session **p_sdp);  
    static void on_rx_rtp(void *user_data, void *pkt, pj_ssize_t size);
    static void on_rx_rtcp(void *user_data, void *pkt, pj_ssize_t size);
    
  };
  
}  // end of namespace

#endif // ifndef
