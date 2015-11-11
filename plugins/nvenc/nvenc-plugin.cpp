/*
 * This file is part of switcher-nvenc.
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

#include "./nvenc-plugin.hpp"
#include "switcher/std2.hpp"
#include "./cuda-context.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    NVencPlugin,
    "nvenc",
    "NVidia video encoder Plugin",
    "video",
    "",
    "CUDA-based video encoder",
    "LGPL",
    "Nicolas Bouillot");

NVencPlugin::NVencPlugin(const std::string &){ 
  auto devices = CudaContext::get_devices();
  std::vector<std::string> names;
  for(auto &it: devices){
    devices_nv_ids_.push_back(it.first);
    names.push_back(std::string("GPU #")
                    + std::to_string(it.first)
                    + " "
                    + it.second);
  }
  if (names.empty())
    return;
  devices_ = Selection(std::move(names), 0);
  update_device();
  pmanage<MPtr(&PContainer::make_selection)>(
      "gpu",
      [this](size_t val){
        if (devices_.get() == val)
          return true;
        devices_.select(val);
        update_device();
        return true;
      },
      [this](){return devices_.get();},
      "encoder GPU",
      "Selection of the GPU used for encoding",
      devices_);
}

bool NVencPlugin::init() {
  return static_cast<bool>(es_.get()->invoke<MPtr(&NVencES::safe_bool_idiom)>());
}

void NVencPlugin::update_device(){
  es_.reset();
  es_ = std2::make_unique<ThreadedWrapper<NVencES>>(devices_nv_ids_[devices_.get()]);
  update_codec();
}

void NVencPlugin::update_codec(){
  codecs_guids_ = es_->invoke<MPtr(&NVencES::get_supported_codecs)>();
  std::vector<std::string> names;
  for(auto &it: codecs_guids_)
    names.push_back(it.first);
  codecs_ = Selection(std::move(names), 0);
  auto set = [this](size_t val){
    if (codecs_.get() != val){
      codecs_.select(val);
      update_preset();
      update_profile();
    }
    return true;
  };
  auto get = [this](){return codecs_.get();};
  if (0 == codecs_id_)
    codecs_id_ = pmanage<MPtr(&PContainer::make_selection)>(
        "codec", set, get, "Codec", "Codec Selection", codecs_);
  else
    pmanage<MPtr(&PContainer::replace)>(
        codecs_id_,
        std2::make_unique<Property2<Selection, Selection::index_t>>(
            set, get, "Codec", "Codec Selection", codecs_, codecs_.size() - 1));
  update_preset();
  update_profile();
}

void NVencPlugin::update_preset(){
  auto cur_codec = codecs_.get_current();
  auto guid_iter = std::find_if(
      codecs_guids_.begin(), codecs_guids_.end(),
      [&](const std::pair<std::string, GUID> &codec){
        return codec.first == cur_codec;
      });
  presets_guids_ = es_->invoke<MPtr(&NVencES::get_presets)>(guid_iter->second);
  std::vector<std::string> names;
  for(auto &it: presets_guids_)
    names.push_back(it.first);
  presets_ = Selection(std::move(names), 0);
  auto set = [this](size_t val){
    if (presets_.get() != val)
      presets_.select(val);
    return true;
  };
  auto get = [this](){return presets_.get();};
  if (0 == presets_id_)
    presets_id_ = pmanage<MPtr(&PContainer::make_selection)>(
        "preset", set, get, "Preset", "Preset Selection", presets_);
  else
    pmanage<MPtr(&PContainer::replace)>(
        presets_id_,
        std2::make_unique<Property2<Selection, Selection::index_t>>(
            set, get, "Preset", "Preset Selection", presets_, presets_.size() - 1));
}

void NVencPlugin::update_profile(){
  auto cur_codec = codecs_.get_current();
  auto guid_iter = std::find_if(
      codecs_guids_.begin(), codecs_guids_.end(),
      [&](const std::pair<std::string, GUID> &codec){
        return codec.first == cur_codec;
      });
  profiles_guids_ = es_->invoke<MPtr(&NVencES::get_profiles)>(guid_iter->second);
  std::vector<std::string> names;
  for(auto &it: profiles_guids_)
    names.push_back(it.first);
  profiles_ = Selection(std::move(names), 0);
  auto set = [this](size_t val){
    if (profiles_.get() != val)
      profiles_.select(val);
    return true;
  };
  auto get = [this](){return profiles_.get();};
  if (0 == profiles_id_)
    profiles_id_ = pmanage<MPtr(&PContainer::make_selection)>(
        "profile", set, get, "Profile", "Profile Selection", profiles_);
  else
    pmanage<MPtr(&PContainer::replace)>(
        profiles_id_,
        std2::make_unique<Property2<Selection, Selection::index_t>>(
            set, get, "Profile", "Profile Selection", profiles_, profiles_.size() - 1));
}

}  // namespace switcher
