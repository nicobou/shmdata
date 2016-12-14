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

#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"

namespace switcher {
/**
 * LADSPA class,
 * Add a LADSPA (Linux Audio Developers Simple Plugin API) plugin among the ones installed on your
 * system.
 */
class LADSPA : public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(LADSPA);

  //! Constructor
  LADSPA(const std::string&);

  //! Destructor
  ~LADSPA() = default;

  //! Mandatory quiddity base class method
  bool init() final;

 private:
  /**
   * The first element is the usage name of each LADSPA plugin.
   * The second one is the name of the gstreamer element, used to create it.
   */
  using PluginList = std::pair<std::vector<std::string>, std::vector<std::string>>;

  static const std::vector<std::string> KPropertiesBlackList;  //!< Blacklisted gstreamer properties

  //! Method used to get all the available ladspa plugins on your system.
  static PluginList get_ladspa_plugins();

  //! Gstreamer pipeline creation methods
  bool create_gst_pipeline();
  void get_gst_properties();

  //! Shmdata methods
  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect();
  bool can_sink_caps(std::string str_caps);

  bool is_valid_{false};   //!< Used to validate that the construction of the quiddity worked
  std::string shmpath_{};  //!< Path of the input shmdata
  std::string shmpath_transformed_{};  //!< Path of the output shmdata
  ShmdataConnector shmcntr_;           //!< Shmdata connector to connect into the quiddity.
  std::unique_ptr<GstPipeliner> gst_pipeline_{nullptr};         //!< Gstreamer pipeline
  std::unique_ptr<GstShmdataSubscriber> shmsrc_sub_{nullptr};   //!< Subscriber to input shmdata
  std::unique_ptr<GstShmdataSubscriber> shmsink_sub_{nullptr};  //!< Subscriber to output shmdata
  bool first_connect_{false};  //!< First connection to a LADSPA plugin?

  GstElement* shmdatasrc_{nullptr};      //!< Gstreamer shmdatasrc element
  GstElement* shmdatasink_{nullptr};     //!< Gstreamer shmdatasink element
  GstElement* ladspa_element_{nullptr};  //!< Gstreamer specific ladspa element
  GstElement* ladspa_bin_{nullptr};      //!< Full ladspa pipeline

  std::vector<std::string> properties_;  //!< Gstreamer properties of the currently selected plugin
  std::map<std::string, std::string>
      saved_properties_;  //!< Properties values for the selected plugin

  PluginList plugins_list_;           //!< List of all LADSPA plugins available on the system
  Selection<> plugins_;               //!< Plugin list property data
  PContainer::prop_id_t plugins_id_;  //!< Plugins list property id
};
};
