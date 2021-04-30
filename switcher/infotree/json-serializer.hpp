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

/**
 * @file   json-serializer.h
 *
 * @brief serialize and deserialize an information tree using json
 *
 *
 */

#ifndef __SWITCHER_INFORMATION_TREE_JSON_H__
#define __SWITCHER_INFORMATION_TREE_JSON_H__

#include <string>
#include "./information-tree.hpp"

namespace switcher {
namespace infotree {
namespace json {

/**
 * Serialize an InfoTree into a JSON string
 * representation.
 *
 * \param tree Tree to serialize
 *
 * \return Serialized string
 */
std::string serialize(InfoTree::ptrc tree);

/**
 * Parse a JSON string and create an Infotree.
 *
 * \param Serialized JSON string to parse
 * \param include_parsing_error If true, then parsing error message
 * will be writen in the tree. The message will be located at
 * the ".parsing_error" branch.
 *
 * \return Tree resulting from the deserialization
 *
 */
InfoTree::ptr deserialize(const std::string& serialized, bool include_parsing_error = false);

}  // namespace json
}  // namespace infotree
}  // namespace switcher
#endif
