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

#ifndef __SWITCHER_GPROP_TO_PROP_H__
#define __SWITCHER_GPROP_TO_PROP_H__

#include <glib.h>
#include <gst/gst.h>
#include <memory>
#include "./property2.hpp"
#include "./std2.hpp"

namespace switcher {
namespace GPropToProp {

std::unique_ptr<PropertyBase> to_prop(GObject* object,
                                      const std::string& gprop_name);

}  // namespace GPropToProp
}  // namespace switcher
#endif
