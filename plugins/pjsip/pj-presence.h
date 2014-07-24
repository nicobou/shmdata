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

#ifndef __SWITCHER_PJ_PRESENCE_H__
#define __SWITCHER_PJ_PRESENCE_H__

#include <mutex>
#include <condition_variable>
#include <pjsua-lib/pjsua.h>
#include <glib.h> //gboolean
#include <glib-object.h>//GEnumValue

namespace switcher
{
  class PJSIP;
  class PJCall;

  class PJPresence
  {
    friend PJSIP;
    friend PJCall; //for account local uri
  public:
    PJPresence () = delete;
    PJPresence (PJSIP *sip_instance);
    ~PJPresence ();
    PJPresence (const PJPresence &) = delete;
    PJPresence &operator= (const PJPresence &) = delete;

    enum {
      AVAILABLE, BUSY, OTP, IDLE, AWAY, BRB, OFFLINE, OPT_MAX
    };
    
  private:
    PJSIP *sip_instance_;
    pjsua_acc_id account_id_;
    std::mutex registration_mutex_;
    std::condition_variable registration_cond_;

    //online status
    GParamSpec *status_enum_spec_;
    static GEnumValue status_enum_[8];
    gint status_;
    GParamSpec *custom_status_spec_;
    std::string custom_status_; 

    //account info
    std::string sip_local_user_;

    //registration
    static  void on_registration_state (pjsua_acc_id acc_id, pjsua_reg_info *info);
    void register_account (const std::string &sip_user, 
			   const std::string &sip_domain, 
			   const std::string &sip_password);
    static gboolean register_account_wrapped (gchar *user, gchar *domain, gchar *password, void *user_data);
    bool unregister_account ();
    static gboolean unregister_account_wrapped (gpointer /*unused*/, void *user_data);
    
    //buddies
    void add_buddy (const std::string &sip_user);
    static void on_buddy_state(pjsua_buddy_id buddy_id);

    //online status
    static void set_status (const gint value, void *user_data);
    static gint get_status (void *user_data);
    static void set_note (const gchar *cutom_status, void *user_data);
    static const gchar *get_note (void *user_data);
    void change_online_status (gint status);

    static void on_reg_state(pjsua_acc_id acc_id);
    static void on_incoming_subscribe(pjsua_acc_id acc_id,
				      pjsua_srv_pres *srv_pres,
				      pjsua_buddy_id buddy_id,
				      const pj_str_t *from,
				      pjsip_rx_data *rdata,
				      pjsip_status_code *code,
				      pj_str_t *reason,
				      pjsua_msg_data *msg_data);

    static void on_buddy_evsub_state(pjsua_buddy_id buddy_id,
				     pjsip_evsub *sub,
				     pjsip_event *event);
  };
}  // end of namespace

#endif // ifndef
