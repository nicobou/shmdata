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

#ifndef __SWITCHER_PJSIP_H__
#define __SWITCHER_PJSIP_H__

#include "switcher/quiddity.h"
#include "switcher/startable-quiddity.h"

#include <pjsua-lib/pjsua.h>
#include <memory>
#include <thread>

namespace switcher
{
  
  class PJSIP : public Quiddity, public StartableQuiddity 
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PJSIP);
    PJSIP ();
    ~PJSIP ();
    PJSIP (const PJSIP &) = delete;
    PJSIP &operator= (const PJSIP &) = delete;
    bool init ();
    bool start ();
    bool stop ();
  private:
    pj_status_t pj_init_status_; 
    pj_thread_desc thread_handler_desc_; 
    pj_thread_t	*pj_thread_ref_; 
    //std::thread sip_init_shutdown_thread_;
    std::thread sip_thread_;
    void sip_init_shutdown_thread ();
    void sip_handling_thread ();
    static void on_buddy_state(pjsua_buddy_id buddy_id);
  };
  
  SWITCHER_DECLARE_PLUGIN(PJSIP);

}  // end of namespace

#endif // ifndef
