/*
 * This file is part of switcher-pjsip.
 *
 * switcher-pjsip is free software: you can redistribute it and/or modify
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

#include "./pj-codec-utils.hpp"
#include <algorithm>
#include <string>
#include <vector>

namespace switcher {
PJCodecUtils::codecs PJCodecUtils::inspect_rtp_codecs() {
  PJCodecUtils::codecs res;
  // assuming gst_init (nullptr, nullptr); has been called
  GList* element_list =
      gst_element_factory_list_get_elements(GST_ELEMENT_FACTORY_TYPE_DEPAYLOADER, GST_RANK_NONE);
  GList* iter = element_list;
  while (iter != nullptr) {
    PJCodecUtils::codecs from_factory = inspect_rtp_codec_from_gst_element_factory(
        reinterpret_cast<GstElementFactory*>(iter->data));
    res.insert(res.end(),
               std::move_iterator<codec_it>(from_factory.begin()),
               std::move_iterator<codec_it>(from_factory.end()));
    iter = g_list_next(iter);
  }
  gst_plugin_feature_list_free(element_list);
  return res;
}

PJCodecUtils::codecs PJCodecUtils::inspect_rtp_codec_from_gst_element_factory(
    GstElementFactory* factory) {
  PJCodecUtils::codecs res;

  const GList* static_pads = gst_element_factory_get_static_pad_templates(factory);

  while (nullptr != static_pads) {
    GstStaticPadTemplate* pad = reinterpret_cast<GstStaticPadTemplate*>(static_pads->data);
    // the following is EMPTY
    // gchar *caps_str = gst_caps_to_string (&pad->static_caps.caps);
    // g_free (caps_str);
    if (nullptr == pad) continue;
    GstCaps* caps =
        gst_static_pad_template_get_caps(pad);  // gst_caps_from_string(pad->static_caps.string);
    // if (!GST_IS_CAPS(caps))
    //   continue;
    PJCodecUtils::codecs from_caps = inspect_rtp_codec_from_gst_caps(caps);

    // replace null "encoding-name" by appropriate
    {
      codec_it not_null_encoding =
          std::find_if(from_caps.begin(), from_caps.end(), [](const RTPCodec::ptr& codec) {
            return 0 != codec->encoding_name_.compare("null");
          });
      if (from_caps.end() != not_null_encoding) {
        std::string encoding = (*not_null_encoding)->encoding_name_;
        for (auto& it : from_caps)
          if (0 == it->encoding_name_.compare("null")) it->encoding_name_ = encoding;
      }
    }
    // move result to res
    res.insert(res.end(),
               std::move_iterator<codec_it>(from_caps.begin()),
               std::move_iterator<codec_it>(from_caps.end()));
    static_pads = g_list_next(static_pads);
    gst_caps_unref(caps);
  }
  return res;
}

std::vector<std::string> PJCodecUtils::get_string_values_from_gst_struct(GstStructure* caps_struct,
                                                                         std::string key) {
  std::vector<std::string> res;
  const GValue* val = gst_structure_get_value(caps_struct, key.c_str());
  if (nullptr != val) {
    if (GST_VALUE_HOLDS_LIST(val)) {
      for (guint i = 0; i < gst_value_list_get_size(val); i++) {
        const GValue* item_val = gst_value_list_get_value(val, i);
        res.emplace_back(g_value_get_string(item_val));
      }
    }
    if (G_VALUE_HOLDS_STRING(val)) {
      res.emplace_back(g_value_get_string(val));
    }
  } else {
    res.emplace_back("null");
  }
  return res;
}

std::vector<gint> PJCodecUtils::get_int_values_from_gst_struct(GstStructure* caps_struct,
                                                               std::string key) {
  std::vector<gint> res;
  const GValue* val = gst_structure_get_value(caps_struct, key.c_str());
  if (nullptr != val) {
    if (GST_VALUE_HOLDS_INT_RANGE(val)) {
      res.push_back(gst_value_get_int_range_max(val));
    }
    if (GST_VALUE_HOLDS_LIST(val)) {
      for (guint i = 0; i < gst_value_list_get_size(val); i++) {
        const GValue* item_val = gst_value_list_get_value(val, i);
        res.push_back(g_value_get_int(item_val));
      }
    }
    if (G_VALUE_HOLDS_INT(val)) {
      res.push_back(g_value_get_int(val));
    }
  }
  return res;
}

PJCodecUtils::codecs PJCodecUtils::inspect_rtp_codec_from_gst_caps(GstCaps* caps) {
  PJCodecUtils::codecs res;
  guint caps_size = gst_caps_get_size(caps);
  if (!gst_caps_is_any(caps)) {
    for (guint i = caps_size; i > 0; i--) {
      GstStructure* caps_struct = gst_caps_get_structure(caps, i - 1);
      if (gst_structure_has_name(caps_struct, "application/x-rtp")) {
        PJCodecUtils::codecs tmp = inspect_rtp_codec_from_gst_struct(caps_struct);
        res.insert(res.begin(),
                   std::move_iterator<codec_it>(tmp.begin()),
                   std::move_iterator<codec_it>(tmp.end()));
      }
    }
  }
  return res;
}

PJCodecUtils::codecs PJCodecUtils::inspect_rtp_codec_from_gst_struct(GstStructure* caps_struct) {
  PJCodecUtils::codecs res;
  // building RTPCodec
  //-- encoding name
  {
    std::vector<std::string> encoding_names =
        get_string_values_from_gst_struct(caps_struct, "encoding-name");
    std::for_each(encoding_names.begin(), encoding_names.end(), [&res](const std::string& str) {
      res.push_back(RTPCodec::ptr(new RTPCodec()));
      res.back()->encoding_name_ = std::move(str);
    });
  }
  //-- payloads
  {
    std::vector<gint> payloads = get_int_values_from_gst_struct(caps_struct, "payload");
    PJCodecUtils::codecs with_payloads;
    std::for_each(payloads.begin(), payloads.end(), [&res, &with_payloads](const gint& pt) {
      PJCodecUtils::codecs this_payload;
      std::for_each(res.begin(), res.end(), [&this_payload, &pt](const RTPCodec::ptr& codec) {
        this_payload.push_back(RTPCodec::ptr(new RTPCodec(*codec)));
        this_payload.back()->payload_ = std::move(pt);
      });
      with_payloads.insert(with_payloads.end(),
                           std::move_iterator<codec_it>(this_payload.begin()),
                           std::move_iterator<codec_it>(this_payload.end()));
    });
    std::swap(res, with_payloads);
  }

  {  //-- media
    std::vector<std::string> media = get_string_values_from_gst_struct(caps_struct, "media");
    PJCodecUtils::codecs with_media;
    std::for_each(media.begin(), media.end(), [&res, &with_media](const std::string media_str) {
      PJCodecUtils::codecs this_media;
      std::for_each(res.begin(), res.end(), [&](const RTPCodec::ptr& codec) {
        this_media.emplace_back(RTPCodec::ptr(new RTPCodec(*codec)));
        this_media.back()->media_ = std::move(media_str);
      });
      with_media.insert(with_media.end(),
                        std::move_iterator<codec_it>(this_media.begin()),
                        std::move_iterator<codec_it>(this_media.end()));
    });
    std::swap(res, with_media);
  }
  {  // clock rate
    std::vector<gint> clock_rates = get_int_values_from_gst_struct(caps_struct, "clock-rate");
    PJCodecUtils::codecs with_clock_rates;
    std::for_each(
        clock_rates.begin(), clock_rates.end(), [&res, &with_clock_rates](const gint rate) {
          PJCodecUtils::codecs this_clock_rate;
          std::for_each(res.begin(), res.end(), [&](const RTPCodec::ptr& codec) {
            this_clock_rate.emplace_back(RTPCodec::ptr(new RTPCodec(*codec)));
            this_clock_rate.back()->clock_rate_ = std::move(rate);
          });
          with_clock_rates.insert(with_clock_rates.end(),
                                  std::move_iterator<codec_it>(this_clock_rate.begin()),
                                  std::move_iterator<codec_it>(this_clock_rate.end()));
        });
    std::swap(res, with_clock_rates);
  }
  return res;
}
}  // namespace switcher
