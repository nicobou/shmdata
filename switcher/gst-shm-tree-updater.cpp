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

#include "./gst-shm-tree-updater.hpp"
#include "./shmdata-utils.hpp"

namespace switcher {

GstShmTreeUpdater::GstShmTreeUpdater(Quiddity* quid,
                                     GstElement* element,
                                     const std::string& shmpath,
                                     Direction dir,
                                     on_caps_cb_t on_caps_cb,
                                     on_delete_t on_delete_cb)
    : quid_(quid),
      shmpath_(shmpath),
      dir_(dir),
      key_(dir_ == Direction::writer ? ".shmdata.writer." : ".shmdata.reader."),
      on_del_(on_delete_cb),
      shm_sub_(element,
               [this, on_caps_cb](const std::string& caps) {
                 auto parent_path = key_ + shmpath_;
                 quid_->graft_tree(parent_path + ".caps", InfoTree::make(caps), false);
                 quid_->graft_tree(parent_path + ".category",
                                   InfoTree::make(ShmdataUtils::get_category(caps)),
                                   false);
                 quid_->notify_tree_updated(parent_path);
                 if (on_caps_cb) on_caps_cb(caps);
               },
               ShmdataStat::make_tree_updater(
                   quid_,
                   key_ + shmpath_,
                   (/* do not signal tree update for readers */ dir_ == Direction::writer
                        ? true
                        : false))) {
  auto path = key_ + shmpath_;
  auto tree = quid_->prune_tree(path, false);
  // adding default informations for this shmdata
  quid_->graft_tree(path, Quiddity::get_shm_information_template(), false);
  if (tree) {
    for (auto& it : tree->get_child_keys(".")) {
      quid_->graft_tree(path + "." + it, tree->prune(it), false);
    }
  }
}

GstShmTreeUpdater::~GstShmTreeUpdater() {
  if (on_del_) on_del_();
  quid_->prune_tree(key_ + shmpath_);
}

}  // namespace switcher
