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

#ifndef __SWITCHER_SELECTION_H__
#define __SWITCHER_SELECTION_H__

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "./types.hpp"

namespace switcher {
namespace quiddity {
namespace property {

/**
 * Selection is a type of property for Quiddity.
 *
 * Selection is a list of named and indexed entries. The Selection API allows for select an entry
 and request the currently selected one.
 *
 * Selection can be done from index or name, but index should be prefered for better performance.
 *
 * Selection entries can also be associated with objects for easier use when working with Selection
 in a Quiddity,
 */
template <typename T = std::string>
class Selection {
 public:
  using index_t = size_t;

  Selection() = delete;
  /**
   * Construct a Selection.
   *
   * \param list      List of name for the entries.
   * \param selection Index of the default selected entry.
   */
  Selection(std::vector<std::string>&& list, index_t selection)
      : list_(list), attached_(list_), current_selection_(selection) {}

  /**
   * Construct a Selection with list of attached paired with the list of names.
   *
   * \param list      Pair of list name with attached object list.
   * \param selection Index of the default selected entry.
   */
  Selection(std::pair<std::vector<std::string> /*names*/, std::vector<T /*attached*/>>&& list,
            index_t selection)
      : list_(std::get<0>(list)), attached_(std::get<1>(list)), current_selection_(selection) {}

  /**
   * Construct a Selection with attached object.
   *
   * \param list      List of all name for each entry.
   * \param attached  List of object attached with entries. The list is expected to have the same
   *                  size as the list of entries.
   * \param selection Index of the default selected entry.
   */
  Selection(std::vector<std::string>&& names, std::vector<T>&& attached, index_t selection)
      : list_(names), attached_(attached), current_selection_(selection) {}

  /**
   * Select an entry.
   *
   * \param new_selection Entry to select.
   */
  void select(IndexOrName new_selection) {
    if (new_selection.is_index_) {
      current_selection_ = new_selection.index_;
    } else {
      current_selection_ = get_name_index(new_selection.name_);
    }
  }

  /**
   * Get current selection.
   *
   * \return Current selection.
   */
  IndexOrName get() const { return IndexOrName(current_selection_, get_current()); }

  /**
   * Get current index.
   *
   * \return Current index.
   */
  size_t get_current_index() const { return current_selection_; }

  /**
   * Get current entry name.
   *
   * \return Current selected name.
   */
  std::string get_current() const { return list_[current_selection_]; }

  /**
   * Get object attached to current selection.
   *
   * \return Object.
   */
  T get_attached() const { return attached_[current_selection_]; }

  /**
   * Get list of entries by name.
   *
   * \return List of entries by name.
   */
  std::vector<std::string> get_list() const { return list_; }

  /**
   * Get an entry index from its name.
   *
   * \param name Entry name.
   *
   * \return Index of the entry.
   */
  index_t get_name_index(const std::string& name) {
    {
      const auto& it = std::find(list_.cbegin(), list_.cend(), name);
      if (it != list_.end()) return it - list_.begin();
    }
    return current_selection_;
  }

  /**
   * Get an entry index from its name or its attached string.
   *
   * \param name_or_attached Entry name or attached string.
   *
   * \return Index of the entry.
   */
  index_t get_index(const std::string& name_or_attached) {
    {
      const auto& it = std::find(list_.cbegin(), list_.cend(), name_or_attached);
      if (it != list_.end()) return it - list_.begin();
    }
    {
      const auto& it = std::find(attached_.cbegin(), attached_.cend(), name_or_attached);
      if (it != attached_.end()) return it - attached_.begin();
    }
    return current_selection_;
  }

  /**
   * Get size of the selection.
   *
   * \return Size.
   **/
  index_t size() const { return list_.size(); }

  /**
   * Request if selection is empty.
   *
   * \return True if empty, false otherwise.
   **/
  bool empty() const { return list_.empty(); }

 private:
  std::vector<std::string> list_;
  std::vector<T> attached_;
  index_t current_selection_{0};
};

}  // namespace property
}  // namespace quiddity
}  // namespace switcher
#endif
