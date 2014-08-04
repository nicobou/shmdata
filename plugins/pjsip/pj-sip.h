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


#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

//pjsip
#include <pjsua-lib/pjsua.h>

#include "switcher/quiddity.h"
#include "switcher/custom-property-helper.h"
#include "pj-call.h"
#include "pj-presence.h"

namespace switcher
{
  class PJSIP : public Quiddity
  {
    friend PJCall;
    friend PJPresence;
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
    CustomPropertyHelper::ptr custom_props_ {std::make_shared<CustomPropertyHelper> ()};
    unsigned sip_port_                      {5060};
    GParamSpec *sip_port_spec_              {nullptr};
    pj_thread_desc thread_handler_desc_     {}; 
    pj_thread_t	*pj_thread_ref_             {nullptr}; 
    pjsua_transport_id *transport_id_       {nullptr};
    std::thread sip_thread_                 {};
    std::mutex pj_init_mutex_               {};
    std::condition_variable pj_init_cond_   {};
    bool pj_sip_inited_                     {false};
    std::mutex work_mutex_                  {};
    std::condition_variable work_cond_      {};
    std::mutex done_mutex_                  {};
    std::condition_variable done_cond_      {};
    bool continue_                          {true};
    std::function<void()> command_          {};
    pj_pool_t *pool_                        {nullptr};
    PJCall *sip_calls_                      {nullptr};
    PJPresence *sip_presence_               {nullptr};
    std::thread sip_worker_                 {};
    bool sip_work_                          {true};
    pj_thread_desc worker_handler_desc_     {}; 
    pj_thread_t	*worker_thread_ref_         {nullptr}; 
    static pjsip_endpoint *sip_endpt_;
    pj_caching_pool cp_;

    void sip_init_shutdown_thread ();
    void sip_handling_thread ();
    bool pj_sip_init ();
    void exit_cmd ();
    void run_command_sync (std::function<void()> command);
    static void set_port (const gint value, void *user_data);
    static gint get_port (void *user_data);
    void sip_worker_thread ();
    void start_udp_transport ();
  };

}  // end of namespace

#endif // ifndef
