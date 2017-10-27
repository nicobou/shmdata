/*
 * This file is part of switcher-ladspa.
 *
 * switcher-ladspa is free software; you can redistribute it and/or
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

#include "./ladspa.hpp"
#include "switcher/gprop-to-prop.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_DECLARE_PLUGIN(LADSPA);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(LADSPA,
                                     "ladspa",
                                     "LADSPA plugin",
                                     "audio",
                                     "reader/writer",
                                     "Choice of LADSPA plugins",
                                     "LGPL",
                                     "Jérémie Soria");

const std::vector<std::string> LADSPA::KPropertiesBlackList = {"name", "parent"};

LADSPA::LADSPA(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      plugins_list_(get_ladspa_plugins()),
      plugins_(Selection<>(std::move(plugins_list_.first), std::move(plugins_list_.second), 0)) {
  if (plugins_list_.first.empty()) return;

  perchannel_group_id_ = pmanage<MPtr(&PContainer::make_group)>(
      "perchannel_group",
      "Per-channel settings",
      "If multiple channels are input they will all be configured separately in this group.");

  global_settings_id_ =
      pmanage<MPtr(&PContainer::make_bool)>("global_settings",
                                            [this](bool val) {
                                              global_settings_ = val;
                                              return true;
                                            },
                                            [this]() { return global_settings_; },
                                            "Settings for all channels",
                                            "Toggle the global settings for all channels",
                                            true);

  plugins_id_ = pmanage<MPtr(&PContainer::make_selection<>)>(
      "plugins",
      [this](const IndexOrName& val) {
        plugins_.select(val);

        // We don't want to reset the properties if they were loaded from a save file
        if (reset_saved_properties_)
          saved_properties_.clear();
        else
          reset_saved_properties_ = true;

        if (!create_gst_pipeline()) return false;
        // Here we just create the pipeline to get the properties, since we don't put the bin in a
        // GstPipeline it
        // will leak if we don't unref.
        gst_object_unref(ladspa_bin_);
        first_connect_ = true;

        return true;
      },
      [this]() { return plugins_.get(); },
      "LADSPA plugin",
      "Select a LADSPA plugin among all the ones installed on the system.",
      plugins_);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return on_shmdata_connect(shmpath); },
      [this](const std::string&) { return on_shmdata_disconnect(); },
      [this]() { return on_shmdata_disconnect(); },
      [this](const std::string& caps) { return can_sink_caps(caps); },
      1);

  pmanage<MPtr(&PContainer::set_to_current)>(plugins_id_);
  pmanage<MPtr(&PContainer::set_to_current)>(global_settings_id_);
}

LADSPA::PluginList LADSPA::get_ladspa_plugins() {
  PluginList plugins_list;
  auto plugins = GstUtils::element_factory_list_to_pair_of_vectors(
      GST_ELEMENT_FACTORY_TYPE_MEDIA_AUDIO, GST_RANK_NONE, false);
  size_t index = 0;
  for (auto& plugin : plugins.second) {
    if (StringUtils::starts_with(plugin, "ladspa")) {
      plugins_list.first.push_back(plugins.first.at(index));
      plugins_list.second.push_back(plugin);
    }
    ++index;
  }

  return plugins_list;
}

void LADSPA::mirror_gst_properties() {
  // Unsubscribe old global properties
  for (auto& subscriber : prop_subscribers_) {
    pmanage<MPtr(&PContainer::unsubscribe)>(subscriber.first, subscriber.second);
  }
  prop_subscribers_.clear();

  // We arbitrarily take the first element as reference
  auto first_element = ladspa_elements_.front();
  if (!first_element) {
    warning("Cannot find a valid ladspa element to mirror properties");
    return;
  }
  std::string first_element_name = gst_element_get_name(first_element);
  for (auto& property : properties_) {
    auto prop_id = static_cast<PContainer::prop_id_t>(pmanage<MPtr(&PContainer::push)>(
        property, GPropToProp::to_prop(G_OBJECT(first_element), property)));

    // When the global property changes we propagate the change on all the elements
    auto rid =
        pmanage<MPtr(&PContainer::subscribe)>(prop_id, [this, property, first_element_name]() {
          if (!global_settings_) return;
          auto property_value =
              pmanage<MPtr(&PContainer::get_str_str)>(property + "_" + first_element_name);
          for (auto& element : ladspa_elements_) {
            pmanage<MPtr(&PContainer::set_str_str)>(property + "_" + gst_element_get_name(element),
                                                    property_value);
          }
        });
    // We start by propagating once for each property.
    pmanage<MPtr(&PContainer::set_to_current)>(prop_id);
    prop_subscribers_.push_back(std::make_pair<>(prop_id, rid));
  }
}

bool LADSPA::create_gst_pipeline() {
  GError* error = nullptr;
  std::string target_format = g_value_get_string(
      GstUtils::get_gst_element_capability(plugins_.get_attached(), "format", GST_PAD_SINK));

  auto target_channels = g_value_get_int(
      GstUtils::get_gst_element_capability(plugins_.get_attached(), "channels", GST_PAD_SINK));

  std::string description =
      std::string("shmdatasrc name=shmsrc ! audioconvert ! audio/x-raw,format=") + target_format;

  if (target_channels > 1) {
    if (channels_number_ != target_channels) {
      warning("LADSPA element % only accepts % channels, we cannot fit % channels in it.",
              plugins_.get_attached(),
              std::to_string(target_channels),
              std::to_string(channels_number_));
      return false;
    }
    description += " ! " + plugins_.get_attached() + " name=ladspa ";
    auto it = saved_properties_.find("ladspa");
    if (it != saved_properties_.end()) {
      for (auto& property : it->second) {
        description += property.first + "=" + property.second + " ";
      }
    }
    description += "! queue ! shmdatasink name=shmsink sync=false";
  } else {
    description += " ! deinterleave name=d interleave name=i ! shmdatasink name=shmsink sync=false";

    for (int channel = 0; channel < channels_number_; ++channel) {
      auto channel_str = std::to_string(channel);
      std::string element_name = std::string("ladspa_") + channel_str;
      description +=
          " d.src_" + channel_str + " ! " + plugins_.get_attached() + " name=" + element_name;
      // We do this in case the pipeline was reset, otherwise the properties values will be reset.
      auto it = saved_properties_.find(element_name);
      if (it != saved_properties_.end()) {
        for (auto& property : it->second) {
          description += std::string(" ") + property.first + "=" + property.second + " ";
        }
      }
      description += " ! queue ! i.sink_" + channel_str;
    }
  }

  auto new_ladspa_bin = gst_parse_bin_from_description(description.c_str(), FALSE, &error);
  bool success = true;
  On_scope_exit {
    if (!success) gst_object_unref(new_ladspa_bin);
  };

  if (error) {
    warning("Could not create ladspa plugin %: %",
            plugins_.get_attached(),
            std::string(error->message));
    g_error_free(error);
    return false;
  }

  auto shmdatasrc = gst_bin_get_by_name(GST_BIN(new_ladspa_bin), "shmsrc");
  auto shmdatasink = gst_bin_get_by_name(GST_BIN(new_ladspa_bin), "shmsink");

  if (!shmdatasrc || !shmdatasink) return false;

  // We do not leak GstElement pointers here since they are in a RAII gstreamer pipeline that
  ladspa_elements_.clear();
  std::vector<GstElement*> ladspa_elements;

  int elements_number = channels_number_;
  if (target_channels > 1) elements_number = 1;
  for (int i = 0; i < elements_number; ++i) {
    std::string element_name = std::string("ladspa_") + std::to_string(i);
    auto ladspa_element = gst_bin_get_by_name(GST_BIN(new_ladspa_bin), element_name.c_str());
    if (!ladspa_element) {
      success = false;
      return false;
    }
    ladspa_elements.push_back(ladspa_element);
  }

  for (auto& element : ladspa_elements) ladspa_elements_.push_back(element);
  shmdatasrc_ = shmdatasrc;
  shmdatasink_ = shmdatasink;
  ladspa_bin_ = new_ladspa_bin;

  get_gst_properties();
  mirror_gst_properties();

  return true;
}

void LADSPA::get_gst_properties() {
  GParamSpec** property_specs;
  unsigned int num_properties;

  if (ladspa_elements_.empty()) return;

  // All LADSPA instances will be of the same plugin so we only keep one vector of properties.
  property_specs =
      g_object_class_list_properties(G_OBJECT_GET_CLASS(ladspa_elements_.back()), &num_properties);
  On_scope_exit { g_free(property_specs); };

  // Remove all the gstreamer properties
  for (auto& element : ladspa_elements_) {
    std::string element_name = gst_element_get_name(element);

    // Remove the group too
    pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>(element_name));

    for (auto& property : properties_) {
      pmanage<MPtr(&PContainer::remove)>(
          pmanage<MPtr(&PContainer::get_id)>(property + "_" + element_name));
    }
  }

  // We unsubscribe the global properties before clearing the list.
  for (auto& property : properties_) {
    pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>(property));
  }
  properties_.clear();

  for (unsigned int i = 0; i < num_properties; ++i) {
    std::string property_name = property_specs[i]->name;

    if (std::find(LADSPA::KPropertiesBlackList.begin(),
                  LADSPA::KPropertiesBlackList.end(),
                  property_name) != LADSPA::KPropertiesBlackList.end())
      continue;

    properties_.push_back(property_name);
  }

  for (auto& element : ladspa_elements_) {
    std::string element_name = gst_element_get_name(element);
    pmanage<MPtr(&PContainer::make_parented_group)>(
        element_name,
        "perchannel_group",
        element_name,
        std::string("Setting of ladspa instance ") + element_name);
    for (auto& property : properties_) {
      pmanage<MPtr(&PContainer::push_parented)>(property + "_" + element_name,
                                                element_name,
                                                GPropToProp::to_prop(G_OBJECT(element), property));
    }
  }
}

bool LADSPA::on_shmdata_connect(const std::string& shmpath) {
  shmpath_ = shmpath;
  shmpath_transformed_ = make_file_name("audio");

  // We get the values before resetting the pipeline before the first connection.
  // After that this will be done during the disconnection event.
  // This is a bit far-fetched but it is necessary due to the fact that we have to create the ladspa
  // element to fetch its properties and we also recreate the pipeline at each connection since it
  // is reset on disconnection. Without this we would lose the property changes between the
  // creation/selection of the plugin and its first connection.
  if (first_connect_) {
    save_properties();
    first_connect_ = false;
  }
  pmanage<MPtr(&PContainer::disable)>(plugins_id_, ShmdataConnector::disabledWhenConnectedMsg);

  create_and_play_gst_pipeline();

  return true;
}

void LADSPA::create_and_play_gst_pipeline() {
  shmsrc_sub_.reset();
  shmsink_sub_.reset();
  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);

  if (!create_gst_pipeline()) return;

  shmsrc_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmdatasrc_,
      [this](const std::string& str_caps) {
        GstCaps* caps = gst_caps_from_string(str_caps.c_str());
        On_scope_exit {
          if (nullptr != caps) gst_caps_unref(caps);
        };
        GstStructure* s = gst_caps_get_structure(caps, 0);
        if (nullptr == s) {
          warning("cannot get structure from caps (ladspa)");
          return;
        }

        gint channels_number = 0;

        if (!gst_structure_get_int(s, "channels", &channels_number)) {
          warning("cannot get channels number from shmdata description (ladspa)");
          return;
        }

        std::lock_guard<std::mutex> lock(channels_change_mutex_);
        if (channels_number != channels_number_) {
          channels_number_ = channels_number;
          channels_change_th_->run_async([this]() {
            std::lock_guard<std::mutex> lock(channels_change_mutex_);
            create_and_play_gst_pipeline();
          });
        }

        graft_tree(
            ".shmdata.reader." + shmpath_,
            ShmdataUtils::make_tree(str_caps, ShmdataUtils::get_category(str_caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.reader." + shmpath_));

  shmsink_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmdatasink_,
      [this](const std::string& caps) {
        graft_tree(".shmdata.writer." + shmpath_transformed_,
                   ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.writer." + shmpath_transformed_),
      [this]() { prune_tree(".shmdata.writer." + shmpath_transformed_); });

  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  g_object_set(G_OBJECT(ladspa_bin_), "async-handling", TRUE, nullptr);
  g_object_set(G_OBJECT(shmdatasrc_), "socket-path", shmpath_.c_str(), nullptr);
  g_object_set(G_OBJECT(shmdatasink_), "socket-path", shmpath_transformed_.c_str(), nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), ladspa_bin_);

  gst_pipeline_->play(true);
}

bool LADSPA::on_shmdata_disconnect() {
  pmanage<MPtr(&PContainer::enable)>(plugins_id_);
  shmsrc_sub_.reset();
  shmsink_sub_.reset();

  // Save the properties values if we reconnect the same plugin.
  save_properties();

  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  return true;
}

bool LADSPA::can_sink_caps(std::string str_caps) {
  if (!StringUtils::starts_with(str_caps, "audio/x-raw")) return false;

  GstCaps* caps = gst_caps_from_string(str_caps.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };

  GstStructure* s = gst_caps_get_structure(caps, 0);
  if (nullptr == s) {
    warning("Cannot get structure from caps (ladspa)");
    return false;
  }

  auto rate = 0;
  if (!gst_structure_get_int(s, "rate", &rate)) {
    warning("Cannot get rate from shmdata description (ladspa)");
    return false;
  }

  auto target_rate =
      GstUtils::get_gst_element_capability_as_range(plugins_.get_attached(), "rate", GST_PAD_SINK);

  return ((target_rate.first <= rate && rate <= target_rate.second));
}

InfoTree::ptr LADSPA::on_saving() {
  auto res = InfoTree::make();

  // We need to refresh the saved properties.
  save_properties();
  for (auto& element : saved_properties_) {
    auto ladspa_key = std::string(".ladspa_properties.") + element.first;
    res->graft(ladspa_key, InfoTree::make());
    for (auto& property : element.second) {
      res->graft(ladspa_key + "." + property.first, InfoTree::make(property.second));
    }
  }
  res->graft(".channels_number", InfoTree::make(channels_number_));
  return res;
}

void LADSPA::on_loading(InfoTree::ptr&& tree) {
  if (!tree || tree->empty()) return;

  auto elements = tree->get_child_keys(".ladspa_properties");

  for (auto& element : elements) {
    auto ladspa_key = std::string(".ladspa_properties.") + element;
    auto properties = tree->get_child_keys(ladspa_key);
    for (auto& property : properties) {
      std::string property_value = tree->branch_read_data<std::string>(ladspa_key + "." + property);
      saved_properties_[element][property] = property_value;
    }
  }

  channels_number_ = tree->branch_read_data<int>(".channels_number");

  reset_saved_properties_ = false;
  pmanage<MPtr(&PContainer::set_to_current)>(plugins_id_);
}

void LADSPA::save_properties() {
  for (auto& element : ladspa_elements_) {
    std::string element_name = gst_element_get_name(element);
    for (auto& prop_name : properties_) {
      saved_properties_[element_name][prop_name] =
          pmanage<MPtr(&PContainer::get_str_str)>(prop_name + "_" + element_name);
    }
  }
}
};
