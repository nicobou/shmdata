/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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


#ifndef __SWITCHER_HTTP_SDP_DEC_H__
#define __SWITCHER_HTTP_SDP_DEC_H__

#include "gpipe.h"
#include "quiddity-command.h"
#include "decodebin-to-shmdata.h"
#include <memory>
#include <map>
#include <string>

namespace switcher
{

  class HTTPSDPDec : public GPipe
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(HTTPSDPDec);
    HTTPSDPDec ();
    ~HTTPSDPDec ();
    HTTPSDPDec (const HTTPSDPDec &) = delete;
    HTTPSDPDec &operator= (const HTTPSDPDec &) = delete;

    bool to_shmdata (std::string uri);

  private: 
   GstElement *souphttpsrc_;
   GstElement *sdpdemux_;
   void init_httpsdpdec ();
   void destroy_httpsdpdec ();
   QuiddityCommand *on_error_command_; //for the pipeline error handler
   std::list<std::unique_ptr<DecodebinToShmdata>> decodebins_;
   void clean_on_error_command ();
   bool init_gpipe () final;
   static void httpsdpdec_pad_added_cb (GstElement* object, 
					GstPad* pad, 
					gpointer user_data);
   static gboolean to_shmdata_wrapped (gpointer uri, 
				       gpointer user_data);
   static void source_setup_cb (GstElement *httpsdpdec, 
				GstElement *source, 
				gpointer user_data);
   static void on_new_element_in_sdpdemux (GstBin     *bin,
                                           GstElement *element,
					   gpointer    user_data);
  };

}  // end of namespace

#endif // ifndef
