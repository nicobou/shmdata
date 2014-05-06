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


#include "pj-codec.h"
#include "pj-call.h"
#include "pj-codec-utils.h"
#include <glib.h> //g_warning
#include <iostream>

namespace switcher
{

  PJCodec::alt_codec_t PJCodec::codec_list[] =
    {
      /* G.729 */
      { { "G729", 4 }, 18, 8000, 1, 10, 8000, 8000 },
      /* PCMU */
      { { "PCMU", 4 }, 0, 8000, 1, 10, 64000, 64000 },
      /* Our proprietary high end low bit rate (5kbps) codec, if you wish */
      { { "FOO", 3 }, 96, 16000, 1, 20, 5000, 5000 },
    };
  
  PJCodec::alt_codec_factory_t PJCodec::alt_codec_factory;
  
  pjmedia_codec_factory_op PJCodec::alt_codec_factory_op =
    {
      &PJCodec::alt_codec_test_alloc,
      &PJCodec::alt_codec_default_attr,
      &PJCodec::alt_codec_enum_codecs,
      &PJCodec::alt_codec_alloc_codec,
      &PJCodec::alt_codec_dealloc_codec,
      &PJCodec::alt_codec_deinit
    };
  
  // PJCodec::PJCodec ()
  // {}
  // PJCodec::~PJCodec ()
  // {}

  pj_status_t PJCodec::alt_codec_test_alloc( pjmedia_codec_factory *factory,
					     const pjmedia_codec_info *id )
  {
    g_print ("*************************************** %s\n", __FUNCTION__);
    unsigned i;
    for (i=0; i<PJ_ARRAY_SIZE(codec_list); ++i) {
      if (pj_stricmp(&id->encoding_name, &codec_list[i].encoding_name)==0)
	return PJ_SUCCESS;
    }
    return PJ_ENOTSUP;
  }
  
  pj_status_t 
  PJCodec::alt_codec_default_attr( pjmedia_codec_factory *factory,
				   const pjmedia_codec_info *id,
				   pjmedia_codec_param *attr )
  {
    g_print ("*************************************** %s\n", __FUNCTION__);
    struct alt_codec *ac;
    unsigned i;

    PJ_UNUSED_ARG(factory);

    for (i=0; i<PJ_ARRAY_SIZE(codec_list); ++i) {
      if (pj_stricmp(&id->encoding_name, &codec_list[i].encoding_name)==0)
	break;
    }
    if (i == PJ_ARRAY_SIZE(codec_list))
      return PJ_ENOTFOUND;

    ac = &codec_list[i];

    pj_bzero(attr, sizeof(pjmedia_codec_param));
    attr->info.clock_rate = ac->clock_rate;
    // attr->info.channel_cnt = ac->channel_cnt;
    // attr->info.avg_bps = ac->avg_bps;
    // attr->info.max_bps = ac->max_bps;
    // attr->info.pcm_bits_per_sample = 16;
    // attr->info.frm_ptime = ac->frm_ptime;
    // attr->info.pt = ac->payload_type;

    // attr->setting.frm_per_pkt = 1;
    // attr->setting.vad = 1;
    // attr->setting.plc = 1;

    return PJ_SUCCESS;
  }

  pj_status_t 
  PJCodec::alt_codec_enum_codecs(pjmedia_codec_factory *factory,
				 unsigned *count,
				 pjmedia_codec_info codecs[])
  {
    g_print ("*************************************** %s\n", __FUNCTION__);

      //TODO get rid of codec_list
    PJCodecUtils::codecs available_codecs = PJCodecUtils::inspect_rtp_codecs ();

    unsigned i = 0;
    for (auto &it : available_codecs)
      {
	std::cout << " encoding " << it->encoding_name_ 
		  << " payload " << it->payload_
		  << " media " << it->media_
		  << " clock rate " << it->clock_rate_
		  << std::endl;
	if (i >= *count)//FIXME
	  break;
	pj_bzero(&codecs[i], sizeof(pjmedia_codec_info));
	//pj_str_t *str;
	pj_cstr (&codecs[i].encoding_name, it->encoding_name_.c_str ()); //FIXME leaking?
	codecs[i].pt = it->payload_;
	if (0 == it->media_.compare ("audio"))
	codecs[i].type = PJMEDIA_TYPE_AUDIO;
	else if (0 == it->media_.compare ("video"))
	   codecs[i].type = PJMEDIA_TYPE_VIDEO;
	 else
	   codecs[i].type = PJMEDIA_TYPE_APPLICATION;
	codecs[i].clock_rate = it->clock_rate_;
	//codecs[i].channel_cnt = ac->channel_cnt;
	codecs[i].channel_cnt = 1;
	i++;
      }
    // unsigned i;
    
    // for (i=0; i<*count && i<PJ_ARRAY_SIZE(codec_list); ++i) {
    //   struct alt_codec *ac = &codec_list[i];
    //   pj_bzero(&codecs[i], sizeof(pjmedia_codec_info));
    //   codecs[i].encoding_name = ac->encoding_name;
    //   codecs[i].pt = ac->payload_type;
    //   codecs[i].type = PJMEDIA_TYPE_AUDIO;
    //   codecs[i].clock_rate = ac->clock_rate;
    //   codecs[i].channel_cnt = ac->channel_cnt;
    // }
    
      *count = i;
      return PJ_SUCCESS;
    }
    
    pj_status_t 
  PJCodec::alt_codec_alloc_codec(pjmedia_codec_factory *factory,
				 const pjmedia_codec_info *id,
				 pjmedia_codec **p_codec)
  {
    g_print ("*************************************** %s\n", __FUNCTION__);
    /* This will never get called since we won't be using this codec */
    //UNIMPLEMENTED(alt_codec_alloc_codec)
    return PJ_ENOTSUP;
  }

  pj_status_t 
  PJCodec::alt_codec_dealloc_codec( pjmedia_codec_factory *factory,
				    pjmedia_codec *codec )
  {
    g_print ("*************************************** %s\n", __FUNCTION__);
    /* This will never get called */
    //UNIMPLEMENTED(alt_codec_dealloc_codec)
    return PJ_ENOTSUP;
  }

  pj_status_t 
  PJCodec::alt_codec_deinit(void)
  {
    g_print ("*************************************** %s\n", __FUNCTION__);
    if (NULL == PJCall::med_endpt_)
      {
	g_warning ("media endpoint is NULL, cannot deinit");
	return PJ_TRUE;//failure 
      }
    pjmedia_codec_mgr *codec_mgr;
    codec_mgr = pjmedia_endpt_get_codec_mgr(PJCall::med_endpt_);
    return pjmedia_codec_mgr_unregister_factory(codec_mgr,
                                                &alt_codec_factory.base);

  }


  pj_status_t 
  PJCodec::install_codecs ()
  {
    g_print ("*************************************** %s\n", __FUNCTION__);
    if (NULL == PJCall::med_endpt_)
      {
	g_warning ("cannot install codec (NULL media endpoint)");
	return PJ_TRUE;//failure
      }

    pjmedia_codec_mgr *codec_mgr;
    pj_status_t status;
    
    /* Register our "dummy" codecs */
    alt_codec_factory.base.op = &alt_codec_factory_op;
    codec_mgr = pjmedia_endpt_get_codec_mgr(PJCall::med_endpt_);
    status = pjmedia_codec_mgr_register_factory(codec_mgr,
						&alt_codec_factory.base);
    if (status != PJ_SUCCESS)
      return status;
    
    /* TODO: initialize your evil library here */
    return PJ_SUCCESS;
  }

}
