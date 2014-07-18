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

namespace switcher
{
  class PJSIP;

  class PJPresence
  {
    friend PJSIP;

  public:
    PJPresence () = delete;
    PJPresence (PJSIP *sip_instance);
    ~PJPresence ();
    PJPresence (const PJPresence &) = delete;
    PJPresence &operator= (const PJPresence &) = delete;

  private:
    PJSIP *sip_instance_;
    pjsua_acc_id account_id_;
    std::mutex registration_mutex_;
    std::condition_variable registration_cond_;
    void register_account (const std::string &sip_user, 
			   const std::string &sip_domain, 
			   const std::string &sip_password);
    static gboolean register_account_wrapped (gchar *user, gchar *domain, gchar *password, void *user_data);
    void add_buddy (const std::string &sip_user);
    static  void on_registration_state (pjsua_acc_id acc_id, pjsua_reg_info *info);
    //buddy
    static void on_buddy_state(pjsua_buddy_id buddy_id);

  };
  
}  // end of namespace

#endif // ifndef
