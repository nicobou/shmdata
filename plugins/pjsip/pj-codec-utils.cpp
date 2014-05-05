/*
 * This file is part of switcher-pjsip.
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

#include "pj-codec-utils.h"
#include <algorithm>

namespace switcher
{
  std::vector<RTPCodec::ptr> 
  PJCodecUtils::inspect_rtp_codecs ()
  {
    std::vector<RTPCodec::ptr> res;
    //assuming gst_init (NULL, NULL); has been called
    GList *element_list = gst_element_factory_list_get_elements (GST_ELEMENT_FACTORY_TYPE_DEPAYLOADER, 
								 GST_RANK_NONE);
    GList *iter = element_list;
    while (iter != NULL)
      {
	// g_print ("+++++\n");
	// g_print ("%s -- ", gst_element_factory_get_longname ((GstElementFactory *)iter->data));
	// g_print ("%s\n", gst_plugin_feature_get_name ((GstPluginFeature *)iter->data));
	std::vector<RTPCodec::ptr> from_factory = 
	  inspect_rtp_codec_from_gst_element_factory ((GstElementFactory *)iter->data);
	std::move (from_factory.begin (),
		   from_factory.end (),
		   res.end());
	iter = g_list_next (iter);
      }
    gst_plugin_feature_list_free (element_list);
    return res;
  }


  std::vector<RTPCodec::ptr>
  PJCodecUtils::inspect_rtp_codec_from_gst_element_factory (GstElementFactory *factory)
  {
    std::vector<RTPCodec::ptr> res;
    
    const GList *static_pads = 
      gst_element_factory_get_static_pad_templates (factory);
    
    while (NULL != static_pads)
      {
	GstStaticPadTemplate *pad = (GstStaticPadTemplate *)static_pads->data; 
	//the following is EMPTY gchar *caps_str = gst_caps_to_string (&pad->static_caps.caps); 
	//g_free (caps_str); 
	/* g_print ("string: %s\n",  */
	/* 	      pad->static_caps.string);  */
	GstCaps *caps = gst_caps_from_string (pad->static_caps.string);
	std::vector<RTPCodec::ptr> from_caps = inspect_rtp_codec_from_gst_caps (caps);
	//FIXME replace null "encoding-name" by appropriate
	std::move (from_caps.begin (),
		   from_caps.end (),
		   res.end());
	static_pads = g_list_next (static_pads); 
	gst_caps_unref (caps);
      }
    
    return res;
  }


  std::vector<std::string> 
  PJCodecUtils::get_string_values_from_gst_struct (GstStructure *caps_struct,
						   std::string key)
  {
    std::vector<std::string> res;
    const GValue *val = gst_structure_get_value (caps_struct, key.c_str ());  
    if (NULL != val) 
      {
	//g_print ("encoding-name struct type %s\n", G_VALUE_TYPE_NAME (val));  
	if (GST_VALUE_HOLDS_LIST(val)) 
	  { 
	    for (guint i = 0; i < gst_value_list_get_size (val); i++) 
	      { 
		const GValue *item_val = gst_value_list_get_value (val, i); 
		//g_print ("encoding-name list %s\n", g_value_get_string (item_val)); 
		res.push_back (g_value_get_string (item_val));
	      } 
	  } 
	if (G_VALUE_HOLDS_STRING (val)) 
	  { 
	    //g_print ("encoding-name string %s\n", g_value_get_string (val)); 
	    res.push_back (g_value_get_string (val));
	  } 
      }
    return res;
  }
  
  std::vector<gint> 
  PJCodecUtils::get_int_values_from_gst_struct (GstStructure *caps_struct,
						std::string key)
  {
    std::vector<gint> res;
    const GValue *val = gst_structure_get_value (caps_struct, key.c_str ());  
    if (NULL != val) 
      { 
	//g_print ("payload struct type %s\n", G_VALUE_TYPE_NAME (val));  
	if(GST_VALUE_HOLDS_INT_RANGE(val)) 
	  { 
	    //g_print ("payload min %d\n", gst_value_get_int_range_max (val));  
	    res.push_back (gst_value_get_int_range_max (val));
	  } 
	if (GST_VALUE_HOLDS_LIST(val)) 
	  { 
	    for (guint i = 0; i < gst_value_list_get_size (val); i++) 
	      { 
		const GValue *item_val = gst_value_list_get_value (val, i); 
		//g_print ("payload list %d\n", g_value_get_int (item_val)); 
		res.push_back (g_value_get_int (item_val));
	      } 
	  } 
	if (G_VALUE_HOLDS_INT (val)) 
	  { 
	    //g_print ("payload int %d\n", g_value_get_int (val)); 
	    res.push_back (g_value_get_int (val));
	  } 
      } 
    return res;
  }

  std::vector<RTPCodec::ptr>
  PJCodecUtils::inspect_rtp_codec_from_gst_caps (GstCaps *caps)
  {
    std::vector<RTPCodec::ptr> FIXMEres;
    guint caps_size = gst_caps_get_size (caps);
    if (! gst_caps_is_any (caps))
      for (guint i = caps_size; i > 0; i--) 
	{
	  GstStructure *caps_struct = gst_caps_get_structure (caps, i-1);
	  if (gst_structure_has_name (caps_struct,"application/x-rtp")) 
	    {
	      // g_print ("string: %s\n",   
	      // 	       gst_structure_to_string (caps_struct));   
	      
	      //FIXME put in a function
	      std::vector<RTPCodec::ptr> res;
  
	      //building RTPCodec
	      //-- encoding name
	      {
		std::vector<std::string> encoding_names =
		  get_string_values_from_gst_struct (caps_struct, "encoding_names");
		std::for_each (encoding_names.begin (), 
			       encoding_names.end (),
			       [&res] (const std::string str) 
			       {
				 auto codec = std::make_shared<RTPCodec> ();
				 codec->encoding_name_ = str;
				 res.push_back (codec);
			       });
	      }
	      //-- payloads
	      {
		std::vector<gint> payloads = 
		  get_int_values_from_gst_struct (caps_struct, "payload");
		std::vector<RTPCodec::ptr> with_payloads;
		std::for_each(payloads.begin (),
			      payloads.end (),
			      [&res, &with_payloads] (const gint pt)
			      {
				std::vector<RTPCodec::ptr> this_payload;
				std::for_each (res.begin (),
					       res.end (),
					       [&] (RTPCodec::ptr codec) 
					       {
						 auto codec_copy = std::make_shared<RTPCodec> (*codec);
						 codec_copy->payload_ = pt;
						 this_payload.push_back (codec_copy);
					       });
				std::move (this_payload.begin (),
					   this_payload.end (),
					   with_payloads.end ());
			      });
		std::swap (res, with_payloads);
	      }
	      {//-- media
		std::vector<std::string> media =
		  get_string_values_from_gst_struct (caps_struct, "media");
		std::vector<RTPCodec::ptr> with_media;
		std::for_each(media.begin (),
			      media.end (),
			      [&res, &with_media] (const std::string media_str)
			      {
				std::vector<RTPCodec::ptr> this_media;
				std::for_each (res.begin (),
					       res.end (),
					       [&] (RTPCodec::ptr codec) 
					       {
						 auto codec_copy = std::make_shared<RTPCodec> (*codec);
						 codec_copy->media_ = media_str;
						 this_media.push_back (codec_copy);
					       });
				std::move (this_media.begin (),
					   this_media.end (),
					   with_media.end ());
			      });
		std::swap (res, with_media);
	      }
	      {//clock rate
		std::vector<gint> clock_rates = 
		  get_int_values_from_gst_struct (caps_struct, 
						  "clock-rate");
		std::vector<RTPCodec::ptr> with_clock_rates;
		std::for_each(clock_rates.begin (),
			      clock_rates.end (),
			      [&res, &with_clock_rates] (const gint rate)
			      {
				std::vector<RTPCodec::ptr> this_clock_rate;
				std::for_each (res.begin (),
					       res.end (),
					       [&] (RTPCodec::ptr codec) 
					       {
						 auto codec_copy = std::make_shared<RTPCodec> (*codec);
						 codec_copy->clock_rate_= rate;
						 this_clock_rate.push_back (codec_copy);
					       });
				std::move (this_clock_rate.begin (),
					   this_clock_rate.end (),
					   with_clock_rates.end ());
			      });
		std::swap (res, with_clock_rates);
	      }
	    } 
	} 
    return FIXMEres;    
  }
}
