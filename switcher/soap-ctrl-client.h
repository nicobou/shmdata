/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
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

#include "quiddity.h"
#include "webservices/soapcontrolProxy.h"
//#include "webservices/control.nsmap"

namespace switcher
{

  class SoapCtrlClient : public Quiddity
  {
  public:
    typedef std::shared_ptr<SoapCtrlClient> ptr;
    bool init ();
    ~SoapCtrlClient ();


    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;


  private:
    controlProxy *switcher_control_;
    gchar *url_;
    static gboolean set_remote_url_wrapped (gpointer url,
					    gpointer user_data);
    static gboolean create_wrapped (gpointer class_name,
				    gpointer quiddity_name,
				    gpointer user_data);
    static gboolean remove_wrapped (gpointer quiddity_name,
				    gpointer user_data);
    static gboolean set_property_wrapped (gpointer quiddity_name,
					  gpointer property_name,
					  gpointer value,
					  gpointer user_data);
    static gboolean invoke1_wrapped (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer user_data);
    static gboolean invoke2_wrapped (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer arg2,
				     gpointer user_data);
    static gboolean invoke3_wrapped (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer arg2,
				     gpointer arg3,
				     gpointer user_data);
    static gboolean invoke4_wrapped (gpointer quiddity_name,
				     gpointer method_name,
				     gpointer arg1,
				     gpointer arg2,
				     gpointer arg3,
				     gpointer arg4,
				     gpointer user_data);

  };
  
}  // end of namespace

#endif // ifndef
