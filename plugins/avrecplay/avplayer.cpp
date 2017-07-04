/*
 * This file is part of switcher-avrecplay.
 *
 * switcher-avrecplay is free software; you can redistribute it and/or
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

#include "avplayer.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(AVPlayer,
                                     "avplayer",
                                     "Shmdata audio video player",
                                     "audio/video",
                                     "writer",
                                     "Replays and controls a recorded shmdata audio/video file",
                                     "LGPL",
                                     "Jérémie Soria");

AVPlayer::AVPlayer(const std::string& name) {}
};
