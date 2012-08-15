/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/connector.h"
#include "switcher/gst-utils.h"

namespace switcher
{

  Connector::Connector () :
    //bin_ (gst_element_factory_make ("bin", NULL)),
    tee_ (gst_element_factory_make ("tee",NULL)),
    input_selector_ (gst_element_factory_make ("input-selector",NULL))
  {
  }
  
  bool
  Connector::set_bin (GstElement *bin)
  {
    
    bin_ = bin;
  
    GstElement *xvimagesink = gst_element_factory_make ("xvimagesink",NULL);
    GstElement *xvimagesink_queue = gst_element_factory_make ("queue",NULL);
    GstElement *fakesink = gst_element_factory_make ("fakesink", NULL);
    GstElement *fakesink_queue = gst_element_factory_make ("queue",NULL);
    GstElement *queue = gst_element_factory_make ("queue",NULL);
        
    gst_bin_add_many (GST_BIN (bin_),
		      input_selector_,
		      queue,
		      tee_,
		      //fakesink_queue,
		      //fakesink,
		      xvimagesink_queue,
		      xvimagesink,
		      NULL);
    gst_element_link_many (input_selector_, queue, tee_, NULL);
    //gst_element_link_many (tee_, fakesink_queue, fakesink, NULL);
    gst_element_link_many (tee_, xvimagesink_queue, xvimagesink, NULL);
    //g_object_set (G_OBJECT (xvimagesink),
    // "sync", 
    // FALSE, 
    // NULL);   
    return true;
  }


  bool
  Connector::connect_to_sink (GstPad *srcpad)
  {
    GstPad *sinkpad = gst_element_get_compatible_pad(input_selector_,
						     srcpad,
						     NULL); //const GstCaps *caps to use as a filter
    
    return GstUtils::check_pad_link_return (gst_pad_link (srcpad, sinkpad));

    // GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sinkpad);
    // gst_element_add_pad (bin_, ghost_sinkpad);
    //return GstUtils::check_pad_link_return (gst_pad_link (srcpad, ghost_sinkpad));
  }

  bool
  Connector::connect_to_sink (GstElement *src_element)
  {

    g_print ("Connector::connect_to_sink\n");
    g_print ("Connector::connect_to_sink - %s\n", GST_ELEMENT_NAME (src_element));
    GstPad *srcpad = gst_element_get_static_pad (src_element, "src");
    
    GstPad *sinkpad = gst_element_get_compatible_pad(input_selector_,
						     srcpad,
						     NULL); //const GstCaps *caps to use as a filter
 
    
    if (! GST_IS_PAD (srcpad))
      {
	g_printerr ("Connector::connect_to_sink - srcpad is not a pad, not linking") ;
	return false;
      }
    if (! GST_IS_PAD (sinkpad))
      {
	g_printerr ("Connector::connect_to_sink - sinkpad is not a pad, not linking") ;
	return false;
      }
    
    // GstPad *ghost_sinkpad = gst_ghost_pad_new (get_name().c_str(), sinkpad);
    // gst_element_add_pad (bin_, ghost_sinkpad);
    // gst_pad_set_active (ghost_sinkpad, TRUE);

    // if (! GST_IS_PAD (ghost_sinkpad))
    //   {
    // 	g_printerr ("Connector::connect_to_sink - sinkpad is not a pad, not linking") ;
    // 	return false;
    //   }

    // bool res = GstUtils::check_pad_link_return (gst_pad_link (srcpad, ghost_sinkpad));
    
    bool res = GstUtils::check_pad_link_return (gst_pad_link (srcpad, sinkpad));

    if (res)
      gst_object_unref (srcpad);
   
    return res;
  }


  //unref after usage
  GstPad *
  Connector::get_src_pad ()
  {
    
    GstElement *queue = gst_element_factory_make ("queue",NULL);
    
    gst_bin_add (GST_BIN (bin_), queue);
    gst_element_sync_state_with_parent (queue);
    gst_element_link (tee_, queue);
    
    GstPad *queue_srcpad = gst_element_get_static_pad (queue, "src");
    return queue_srcpad;

    // GstPad *ghost_srcpad = gst_ghost_pad_new (NULL, queue_srcpad);
    // gst_pad_set_active(ghost_srcpad,TRUE);
    // gst_element_add_pad (bin_, ghost_srcpad); 

    // gst_object_unref (queue_srcpad);
    // return ghost_srcpad;
  }
  

  //sink_element must be *in* the segment bin
  bool 
  Connector::connect_to_src (GstElement *sink_element)
  {
     GstPad *srcpad = get_src_pad ();
     GstPad *sinkpad = gst_element_get_compatible_pad(sink_element,
     						     srcpad,
     						     NULL); //const GstCaps *caps to use as a filter

     //gst_element_get_static_pad (sink_element, "sink");

     if (! GST_IS_PAD (srcpad))
       {
     	g_printerr ("Connector::connect_to_src - srcpad is not a pad, not linking") ;
     	return false;
       }
     if (! GST_IS_PAD (sinkpad))
       {
     	g_printerr ("Connector::connect_to_src - sinkpad is not a pad, not linking") ;
     	return false;
       }
    
     GstPadLinkReturn linkret = gst_pad_link (srcpad, sinkpad);
     gst_object_unref (sinkpad);
     gst_object_unref (srcpad);

     return GstUtils::check_pad_link_return (linkret);
  }

}  // end of namespace
