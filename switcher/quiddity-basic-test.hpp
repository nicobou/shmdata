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

#ifndef __SWITCHER_QUIDDITY_BASIC_TEST_H__
#define __SWITCHER_QUIDDITY_BASIC_TEST_H__

namespace switcher {
class QuiddityBasicTest {
 public:
  static bool test_full(QuiddityManager::ptr manager,
                        const std::string &quiddity_class_name);
  static bool test_get_info(QuiddityManager::ptr manager,
                            const std::string &quiddity_class_name);
  static bool test_create(QuiddityManager::ptr manager,
                          const std::string &quiddity_class_name);
  static bool test_description_by_class(QuiddityManager::ptr manager,
                                        const std::string &quiddity_class_name);
  static bool test_startable(QuiddityManager::ptr manager,
                             const std::string &quiddity_class_name);
};

}  // namespace switcher
#endif
