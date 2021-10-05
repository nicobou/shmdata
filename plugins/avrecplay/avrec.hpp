/*
 * This file is part of switcher-avrec.
 *
 * switcher-avrec is free software; you can redistribute it and/or
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

#ifndef SWITCHER_AVREC_HPP
#define SWITCHER_AVREC_HPP

#include <switcher/quiddity/startable.hpp>
#include "switcher/gst/pipeliner.hpp"
#include "switcher/shmdata/follower.hpp"

namespace switcher {
namespace quiddities {

using namespace quiddity;
class AVRecorder : public Quiddity, public Startable {
 public:
  AVRecorder(quiddity::Config&&);
  bool start() final;
  bool stop() final;

 private:
  static const std::string kConnectionSpec;  //!< Shmdata specifications

  static std::vector<std::string> kGstMuxers;                  //!< List of GStreamer muxers
  static const std::vector<std::string> kPropertiesBlackList;  //!< Blacklisted gstreamer properties
  static const std::string kRecordModeDate;
  static const std::string kRecordModeLabel;
  static const std::string kRecordModeOverwrite;

  GstBusSyncReply bus_sync(GstMessage* msg);

  //! Shmdata methods
  bool on_shmdata_connect(const std::string& shmpath, claw::sfid_t sfid);
  bool on_shmdata_disconnect(claw::sfid_t sfid);

  std::string generate_pipeline_description();

  //! property::Property custom saving
  InfoTree::ptr on_saving() final;
  void on_loading(InfoTree::ptr&& tree) final;
  void save_properties();

  struct ElementProperties {
    GstElement* elem{nullptr};
    std::vector<std::string> properties{};
  };

  struct ConnectedShmdata {
    ConnectedShmdata(AVRecorder* parent, const std::string& shmpath, claw::sfid_t sfid);
    ~ConnectedShmdata();
    void discover_compatible_muxers(std::vector<std::string>&);
    bool update_gst_properties();
    void create_label_property();
    void apply_gst_properties(GstElement* element);

    AVRecorder* parent_{nullptr};  //!< AVRecorder parent, initialized at construction.
    std::string caps_{};           //!< String caps received on connection of the shmdata.
    std::string shmdata_name_{};   //!< Extraction of the unique shmdata name from the shmpath.
    std::string shmpath_{};        //!< Shmdata file path.
    claw::sfid_t sfid_{Ids::kInvalid};  //!< the claw identifier for thethis shmdata follower
    std::string recfile_{};        //!< Name of the recording file (modulo suffix from record mode).
    property::prop_id_t recfile_id_{0};
    std::string label_{"custom"};  //!< Label suffix for label record mode.
    property::prop_id_t label_id_{0};
    //!< List of all supported muxers for the shmdata and their respective properties.
    std::map<std::string, ElementProperties> muxers_{};
    //!< Shmdata follower used to detect supported muxers for the shmdata.
    std::unique_ptr<shmdata::Follower> shm_follower_{nullptr};
    property::Selection<> muxer_selection_{{"none"}, 0};  //!< Supported muxer selection.
    property::prop_id_t muxer_selection_id_{0};
    std::vector<property::prop_id_t> muxer_properties_id_{};
  };

  bool is_valid_{false};      //!< Used to validate that the construction of the quiddity worked
  std::unique_ptr<gst::Pipeliner> gst_pipeline_{nullptr};  //!< Gstreamer pipeline
  GstElement* avrec_bin_{nullptr};                       //!< Full recording pipeline

  std::mutex eos_m_{};
  std::condition_variable cv_eos_{};

  std::vector<std::unique_ptr<ConnectedShmdata>> connected_shmdata_{};

  //! Quiddity properties
  std::string recpath_{};                //!< Path where the shmdata will be recorded.
  property::prop_id_t recpath_id_{0};    //!< property::Property id of the recording path.
  //!< property::Selection of recording modes, can be by date (suffix date at each recording), by
  //!< label
  //!(suffix cusotm label at each recording) or overwrite (rewrite the file if it already exists).
  //! Defaults to date.
  property::Selection<> record_mode_{{kRecordModeDate, kRecordModeLabel, kRecordModeOverwrite}, 0};
  property::prop_id_t record_mode_id_{0};  //!< property::Property id of the record mode selection.
  std::map<std::string, std::map<std::string, std::string>>
      saved_properties_;  //!< Properties values of the selected muxer for all connected shmdata.
};

}  // namespace quiddities
}  // namespace switcher

#endif
