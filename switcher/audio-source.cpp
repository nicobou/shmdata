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

#include "audio-source.h"
#include "gst-utils.h"
#include "gst-element-cleaner.h"
#include <memory>

namespace switcher
{
  AudioSource::AudioSource() 
  { 
    if (!GstUtils::make_element ("tee", &audio_tee_)
	|| !GstUtils::make_element ("audioconvert", &audioconvert_)
	|| !GstUtils::make_element ("pitch", &pitch_)
	|| !GstUtils::make_element ("audioresample", &resample_))
      g_critical ("a mandatory gst element is missing for audio source");

    cleaner_.reset (new GstElementCleaner ());
    cleaner_->add_element_to_cleaner (audio_tee_);
    cleaner_->add_element_to_cleaner (audioconvert_);
    cleaner_->add_element_to_cleaner (pitch_);
    cleaner_->add_element_to_cleaner (resample_);
    
    
    gst_bin_add_many (GST_BIN (bin_),
		      audio_tee_,
		      audioconvert_,
		      pitch_,
		      resample_,
		      NULL);
    gst_element_link_many (audio_tee_,
			   audioconvert_,
			   pitch_,
			   resample_,
			   NULL);
    
    register_property (G_OBJECT (pitch_),"output-rate","pitch-ouput-rate");
    register_property (G_OBJECT (pitch_),"rate","pitch-rate");
    register_property (G_OBJECT (pitch_),"tempo","pitch-tempo");
    register_property (G_OBJECT (pitch_),"pitch","pitch");

  }

  void 
  AudioSource::set_raw_audio_element (GstElement *elt)
  {
    rawaudio_ = elt;
 
    gst_bin_add (GST_BIN (bin_), rawaudio_);
    gst_element_link (rawaudio_, audio_tee_);

    GstCaps *audiocaps = gst_caps_new_simple ("audio/x-raw-float",
					      "width", G_TYPE_INT, 32,
     					      NULL);

    //creating a connector for raw audio
    ShmdataWriter::ptr rawaudio_connector;
    rawaudio_connector.reset (new ShmdataWriter ());
    std::string rawconnector_name = make_file_name ("rawaudio");
    rawaudio_connector->set_path (rawconnector_name.c_str());
    rawaudio_connector->plug (bin_, audio_tee_, audiocaps);
    register_shmdata_writer (rawaudio_connector);
    
    //creating a connector for transformed audio
    ShmdataWriter::ptr audio_connector;
    audio_connector.reset (new ShmdataWriter ());
    std::string connector_name = make_file_name ("audio");
    audio_connector->set_path (connector_name.c_str());
    audio_connector->plug (bin_, audio_tee_, audiocaps);
    register_shmdata_writer (audio_connector);
    
    //gst_object_unref (audiocaps);
    gst_caps_unref(audiocaps);
  }

}
