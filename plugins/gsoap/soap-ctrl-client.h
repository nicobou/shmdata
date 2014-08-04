/*
 * This file is part of switcher-gsoap.
 *
 * switcher-gsoap is free software: you can redistribute it and/or modify
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


#ifndef __SWITCHER_SOAP_CTRL_CLIENT_H__
#define __SWITCHER_SOAP_CTRL_CLIENT_H__

#include <mutex>

#include "switcher/quiddity.h"
#include "webservices/soapcontrolProxy.h"

namespace switcher
{

  class SoapCtrlClient : public Quiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(SoapCtrlClient);
    SoapCtrlClient ();
    ~SoapCtrlClient ();
    SoapCtrlClient (const SoapCtrlClient &) = delete;
    SoapCtrlClient &operator= (const SoapCtrlClient &) = delete;
    bool init () final;

  private:
    controlProxy *switcher_control_;
    gchar *url_;
    GSource *try_connect_g_source_;
    std::mutex try_connect_mutex_;
    void reset_endpoint ();
    static gboolean set_remote_url (gpointer url,
					    gpointer user_data);
    static gboolean set_remote_url_retry (gpointer url,
					  gpointer user_data);
    static gboolean try_connect (gpointer user_data);

    static gboolean create (gpointer class_name,
				    gpointer quiddity_name,
				    gpointer user_data);
    static gboolean remove (gpointer quiddity_name,
				    gpointer user_data);
    static gboolean set_property (gpointer quiddity_name,
					  gpointer property_name,
					  gpointer value,
					  gpointer user_data);
    static gboolean invoke1 (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer user_data);
    static gboolean invoke2 (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer arg2,
				     gpointer user_data);
    static gboolean invoke3 (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer arg2,
				     gpointer arg3,
				     gpointer user_data);
    static gboolean invoke4 (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer arg2,
				     gpointer arg3,
				     gpointer arg4,
				     gpointer user_data);

  };

  SWITCHER_DECLARE_PLUGIN(SoapCtrlClient);
  
}  // end of namespace

#endif // ifndef
