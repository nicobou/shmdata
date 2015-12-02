/*
 * This file is part of switcher-pjsip.
 *
 * switcher-pjsip is free software: you can redistribute it and/or modify
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

#include <pjnath.h>
#include <glib.h>
#include "./pj-sip-plugin.hpp"
#include "./pj-stun-turn.hpp"

namespace switcher {
PJStunTurn::PJStunTurn(){
  if (PJ_SUCCESS != pjnath_init()){
    g_warning("cannot init pjnath");
    return;
  }
  pj_ice_strans_cfg_default(&ice_cfg_);
  pj_timer_heap_create(PJSIP::this_->pool_, 100, 
                       &ice_cfg_.stun_cfg.timer_heap);
  pj_ioqueue_create(PJSIP::this_->pool_, 128, 
                    &ice_cfg_.stun_cfg.ioqueue);
  if (PJ_SUCCESS != pj_thread_create(PJSIP::this_->pool_,
                                    "switcherSIP",
                                    &worker_thread,
                                    this,
                                    0,
                                    0,
                                     &thread_)){
        g_warning("STUN TURN thread creating failled");
        return;
      }
      
  ice_cfg_.af = pj_AF_INET();
  ice_cfg_.opt.aggressive = PJ_TRUE;

  // set stun/turn config
  SIPPlugin::this_->
      install_method("Set STUN/TURN parameters",  // long name
                     "set_stun_turn",  // name
                     "Set STUN/TURN configuration",  // description
                     "the server(s) are reachable",  // return description
                     Method::make_arg_description("STUN server address",  // long name
                                                  "stun",  // name
                                                  "string",  // description
                                                  "TURN server address",
                                                  "turn",
                                                  "string",
                                                  "TURN user name",
                                                  "turn_user",
                                                  "string",
                                                  "TURN user password",
                                                  "turn_pass",
                                                  "string",
                                                  nullptr),
                     (Method::method_ptr)&set_stun_turn,
                     G_TYPE_BOOLEAN,
                     Method::make_arg_type_description(G_TYPE_STRING,
                                                       G_TYPE_STRING,
                                                       G_TYPE_STRING,
                                                       G_TYPE_STRING,
                                                       nullptr),
                     this);
  
}

PJStunTurn::~PJStunTurn() {
  worker_quit_ = true;
  if (nullptr != thread_) {
    pj_thread_join(thread_);
    pj_thread_destroy(thread_);
  }
  if (ice_cfg_.stun_cfg.ioqueue)
    pj_ioqueue_destroy(ice_cfg_.stun_cfg.ioqueue);
  
  if (ice_cfg_.stun_cfg.timer_heap)
    pj_timer_heap_destroy(ice_cfg_.stun_cfg.timer_heap);
}

int PJStunTurn::worker_thread(void *data)
{
  auto context = static_cast<PJStunTurn *>(data);
  while (!context->worker_quit_) {
    context->handle_events(500, NULL);
  }
  return 0;
}

pj_status_t PJStunTurn::handle_events(unsigned max_msec, unsigned *p_count)
{
  enum { MAX_NET_EVENTS = 1 };
  pj_time_val max_timeout = {0, 0};
  pj_time_val timeout = { 0, 0};
  unsigned count = 0, net_event_count = 0;
  int c;
  max_timeout.msec = max_msec;
  /* Poll the timer to run it and also to retrieve the earliest entry. */
  timeout.sec = timeout.msec = 0;
  c = pj_timer_heap_poll( ice_cfg_.stun_cfg.timer_heap, &timeout );
  if (c > 0)
    count += c;
  /* timer_heap_poll should never ever returns negative value, or otherwise
   * ioqueue_poll() will block forever!
   */
  pj_assert(timeout.sec >= 0 && timeout.msec >= 0);
  if (timeout.msec >= 1000) timeout.msec = 999;
  /* compare the value with the timeout to wait from timer, and use the 
   * minimum value. 
   */
  if (PJ_TIME_VAL_GT(timeout, max_timeout))
    timeout = max_timeout;
  /* Poll ioqueue. 
   * Repeat polling the ioqueue while we have immediate events, because
   * timer heap may process more than one events, so if we only process
   * one network events at a time (such as when IOCP backend is used),
   * the ioqueue may have trouble keeping up with the request rate.
   *
   * For example, for each send() request, one network event will be
   *   reported by ioqueue for the send() completion. If we don't poll
   *   the ioqueue often enough, the send() completion will not be
   *   reported in timely manner.
   */
  do {
    c = pj_ioqueue_poll( ice_cfg_.stun_cfg.ioqueue, &timeout);
    if (c < 0) {
      pj_status_t err = pj_get_netos_error();
      pj_thread_sleep(PJ_TIME_VAL_MSEC(timeout));
      if (p_count)
        *p_count = count;
      return err;
    } else if (c == 0) {
      break;
    } else {
      net_event_count += c;
      timeout.sec = timeout.msec = 0;
    }
  } while (c > 0 && net_event_count < MAX_NET_EVENTS);
  count += net_event_count;
  if (p_count)
    *p_count = count;
  return PJ_SUCCESS;
  
}

gboolean PJStunTurn::set_stun_turn(gchar *stun,
                                   gchar *turn,
                                   gchar *turn_user,
                                   gchar *turn_pass,
                                   void *user_data) {
  //PJStunTurn *context = static_cast<PJStunTurn *>(user_data);

  // saving old config for reverting if error with the new one to be tested:
  //pj_ice_strans_cfg old_ice_cfg = context->ice_cfg_;

  // /* Configure STUN/srflx candidate resolution */
  //   if (icedemo.opt.stun_srv.slen) {
  //       char *pos;

  //       /* Command line option may contain port number */
  //       if ((pos=pj_strchr(&icedemo.opt.stun_srv, ':')) != NULL) {
  //           icedemo.ice_cfg.stun.server.ptr = icedemo.opt.stun_srv.ptr;
  //           icedemo.ice_cfg.stun.server.slen = (pos - icedemo.opt.stun_srv.ptr);

  //           icedemo.ice_cfg.stun.port = (pj_uint16_t)atoi(pos+1);
  //       } else {
  //           icedemo.ice_cfg.stun.server = icedemo.opt.stun_srv;
  //           icedemo.ice_cfg.stun.port = PJ_STUN_PORT;
  //       }

  //       /* For this demo app, configure longer STUN keep-alive time
  //        * so that it does't clutter the screen output.
  //        */
  //       icedemo.ice_cfg.stun.cfg.ka_interval = KA_INTERVAL;
  //   }

  //   /* Configure TURN candidate */
  //   if (icedemo.opt.turn_srv.slen) {
  //       char *pos;

  //       /* Command line option may contain port number */
  //       if ((pos=pj_strchr(&icedemo.opt.turn_srv, ':')) != NULL) {
  //           icedemo.ice_cfg.turn.server.ptr = icedemo.opt.turn_srv.ptr;
  //           icedemo.ice_cfg.turn.server.slen = (pos - icedemo.opt.turn_srv.ptr);

  //           icedemo.ice_cfg.turn.port = (pj_uint16_t)atoi(pos+1);
  //       } else {
  //           icedemo.ice_cfg.turn.server = icedemo.opt.turn_srv;
  //           icedemo.ice_cfg.turn.port = PJ_STUN_PORT;
  //       }

  //       /* TURN credential */
  //       icedemo.ice_cfg.turn.auth_cred.type = PJ_STUN_AUTH_CRED_STATIC;
  //       icedemo.ice_cfg.turn.auth_cred.data.static_cred.username = icedemo.opt.turn_username;
  //       icedemo.ice_cfg.turn.auth_cred.data.static_cred.data_type = PJ_STUN_PASSWD_PLAIN;
  //       icedemo.ice_cfg.turn.auth_cred.data.static_cred.data = icedemo.opt.turn_password;

  //       /* Connection type to TURN server */
  //       if (icedemo.opt.turn_tcp)
  //           icedemo.ice_cfg.turn.conn_type = PJ_TURN_TP_TCP;
  //       else
  //           icedemo.ice_cfg.turn.conn_type = PJ_TURN_TP_UDP;

  //       /* For this demo app, configure longer keep-alive time
  //        * so that it does't clutter the screen output.
  //        */
  //       icedemo.ice_cfg.turn.alloc_param.ka_interval = KA_INTERVAL;
  //   }

  return TRUE;
}



}  // namespace switcher
