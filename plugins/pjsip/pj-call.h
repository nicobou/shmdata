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


namespace switcher
{
  class PJSIP;

  class PJCall
  {
  public:
    PJCall () = delete;
    PJCall (PJSIP *sip_instance);
    ~PJCall ();
    PJCall (const PJCall &) = delete;
    PJCall &operator= (const PJCall &) = delete;
  private:
    pjmedia_endpt *med_endpt_;
    static pjsip_module mod_siprtp_;
    static pj_bool_t on_rx_request (pjsip_rx_data *rdata);
    static void call_on_state_changed( pjsip_inv_session *inv, 
				       pjsip_event *e);
    static void call_on_forked(pjsip_inv_session *inv, pjsip_event *e);
    static void call_on_media_update( pjsip_inv_session *inv,
				      pj_status_t status);
  };
  
}  // end of namespace

#endif // ifndef
