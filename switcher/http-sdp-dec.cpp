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

#include "http-sdp-dec.h"
#include "gst-utils.h"
#include "scope-exit.h"
#include <glib/gprintf.h>
#include <memory>

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(HTTPSDPDec,
				       "HTTP/SDP Decoder",
				       "network source", 
				       "decode an sdp-described stream deliver through http and make shmdatas",
				       "LGPL",
				       "httpsdpdec", 
				       "Nicolas Bouillot");
  
  HTTPSDPDec::HTTPSDPDec () :
    souphttpsrc_ (nullptr),
    sdpdemux_ (nullptr),
    on_error_command_ (nullptr),
    decodebins_ ()
  {}

  HTTPSDPDec::~HTTPSDPDec ()
  {
    destroy_httpsdpdec ();
  }
  
  bool
  HTTPSDPDec::init_gpipe () 
  { 
    if (!GstUtils::make_element ("souphttpsrc", &souphttpsrc_)
	|| !GstUtils::make_element ("sdpdemux", &sdpdemux_))
      return false;
    destroy_httpsdpdec ();
 
    install_method ("To Shmdata",
		    "to_shmdata", 
		    "get raw streams from an sdp description distributed over http and write them to shmdatas", 
		    "success or fail",
		    Method::make_arg_description ("URL",
						  "url", 
						  "the url to the sdp file",
						  nullptr),
		    (Method::method_ptr) &to_shmdata_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);
    return true;
  }

  void 
  HTTPSDPDec::init_httpsdpdec ()
  {
    if (!GstUtils::make_element ("souphttpsrc", &souphttpsrc_)
	|| !GstUtils::make_element ("sdpdemux", &sdpdemux_))
      {
	g_warning ("HTTPSDPDec::init_httpsdpdec, cannot create httpsdpdec elements");
	return;
      }

    g_signal_connect (GST_BIN (sdpdemux_),  
		      "element-added",  
		      (GCallback) HTTPSDPDec::on_new_element_in_sdpdemux,  
		      nullptr);      

    g_object_set (G_OBJECT (sdpdemux_), 
      		  "latency",
      		  0,
      		  nullptr);
    g_object_set (G_OBJECT (sdpdemux_), 
     		  "async-handling",
     		  TRUE,
     		  nullptr);
    g_signal_connect (G_OBJECT (sdpdemux_), 
		      "pad-added", 
		      (GCallback) HTTPSDPDec::httpsdpdec_pad_added_cb,
		      (gpointer) this);
  }

  void 
  HTTPSDPDec::destroy_httpsdpdec ()
  {
    clean_on_error_command ();
    clear_shmdatas ();
    reset_bin ();
    reset_counter_map ();
    souphttpsrc_ = nullptr;
    sdpdemux_ = nullptr;
    
  }

  void 
  HTTPSDPDec::on_new_element_in_sdpdemux (GstBin     *bin,
					  GstElement *element,
					  gpointer    user_data)
  {
    //FIXME add that in uridecodebin 
    g_object_set (G_OBJECT (element), 
      		  "ntp-sync",
      		  TRUE,
      		  nullptr);
  }
  
  void 
  HTTPSDPDec::clean_on_error_command ()
  {
    if (nullptr != on_error_command_)
      {
	delete on_error_command_;
	on_error_command_ = nullptr;
      }
  }

  void 
  HTTPSDPDec::httpsdpdec_pad_added_cb (GstElement */*object*/, 
				       GstPad *pad, 
				       gpointer user_data)   
  {   
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    GPipe *gpipe = static_cast<GPipe *>(user_data);
    std::unique_ptr<DecodebinToShmdata> decodebin (new DecodebinToShmdata (*gpipe));

    decodebin->invoke (std::bind (gst_bin_add,
				 GST_BIN (context->bin_),
				 std::placeholders::_1));
    
    // GstPad *sinkpad = gst_element_get_static_pad (decodebin, "sink");
    auto get_pad = std::bind (gst_element_get_static_pad, 
			      std::placeholders::_1, 
			      "sink");
    GstPad *sinkpad = decodebin->invoke_with_return<GstPad *> (std::move (get_pad));
    On_scope_exit {gst_object_unref (GST_OBJECT (sinkpad));};

    GstUtils::check_pad_link_return (gst_pad_link (pad, sinkpad));
    decodebin->invoke (std::bind (GstUtils::sync_state_with_parent,
				  std::placeholders::_1));
    
    context->decodebins_.push_back (std::move (decodebin)); 
  }   

  void 
  HTTPSDPDec::source_setup_cb (GstElement */*httpsdpdec*/, 
			       GstElement *source, 
			       gpointer /*user_data*/)
  {
    //HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    g_debug ("source %s %s\n",  GST_ELEMENT_NAME(source), G_OBJECT_CLASS_NAME (G_OBJECT_GET_CLASS (source)));
  }

  gboolean
  HTTPSDPDec::to_shmdata_wrapped (gpointer uri, 
				    gpointer user_data)
  {
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  
    if (context->to_shmdata ((char *)uri))
      return TRUE;
    else
      return FALSE;
  }

  bool
  HTTPSDPDec::to_shmdata (std::string uri)
  {
    destroy_httpsdpdec ();
    reset_bin ();
    init_httpsdpdec ();
    clean_on_error_command ();
    on_error_command_ = new QuiddityCommand ();
    on_error_command_->id_ = QuiddityCommand::invoke;
    on_error_command_->time_ = 1000; // 1 second
    on_error_command_->add_arg (get_nick_name ());
    on_error_command_->add_arg ("to_shmdata");
    std::vector<std::string> vect_arg;
    vect_arg.push_back (uri);
    on_error_command_->set_vector_arg (vect_arg);

    g_object_set_data (G_OBJECT (sdpdemux_), 
     		       "on-error-command",
     		       (gpointer)on_error_command_);
    
    g_debug ("httpsdpdec: to_shmdata set uri %s", uri.c_str ());

    g_object_set (G_OBJECT (souphttpsrc_), "location", uri.c_str (), nullptr); 

    gst_bin_add_many (GST_BIN (bin_), 
		      souphttpsrc_, 
		      sdpdemux_,
		      nullptr);
    
    gst_element_link (souphttpsrc_, sdpdemux_);

    GstUtils::sync_state_with_parent (souphttpsrc_);
    GstUtils::sync_state_with_parent (sdpdemux_);
    return true;
  }


}
