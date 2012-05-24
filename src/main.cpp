/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "switcher/controller.h"
#include "switcher/runtime.h"
#include "switcher/base-entity.h"
#include "switcher/creator.h"
#include "switcher/video-test-source.h"
#include <iostream>


int
main (int argc,
      char *argv[])
{
  using namespace switcher;
  
  //Factory + registration of base entity
  Factory<BaseEntity, std::string> temp;
  temp.Register<Runtime> ("runtime");
  temp.Register<VideoTestSource> ("videotestsource");

  //Create and call
  BaseEntity::ptr runtime = temp.Create ("runtime");
  //printf("Runtime %u\n", runtime->Get());
  Runtime::ptr rt = std::tr1::dynamic_pointer_cast<Runtime> (runtime);
  
  BaseEntity::ptr videotest = temp.Create("videotestsource");
  VideoTestSource::ptr seg = std::tr1::dynamic_pointer_cast<VideoTestSource> (videotest); 
  seg->set_runtime (rt); //..and play
  
  // BaseEntity::ptr myvideotest = temp.Create("videotestsource");
  // VideoTestSource::ptr myseg = std::tr1::dynamic_pointer_cast<VideoTestSource> (myvideotest); 
  // rt->add_segment (myseg);

  

  rt->run();
  
  return 0;
}
