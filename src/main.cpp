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

#include "soapswitcher_controlService.h"
#include "switcher_control.nsmap"


int
main (int argc,
      char *argv[])
{
  using namespace switcher;
  
  //Factory + registration of base entity
  Factory<BaseEntity, std::string> temp;
  temp.Register<Runtime> ("runtime");
  temp.Register<VideoTestSource> ("videotestsource");

  switcher_controlService switcher_control;
  if (argc < 2)
    switcher_control.serve();	/* serve as CGI application */
  else
  { int port = atoi(argv[1]);
    if (!port)
    { fprintf(stderr, "Usage: switcher <port>\n");
      exit(0);
    }
    /* run iterative server on port until fatal error */
    if (switcher_control.run(port))
    { switcher_control.soap_stream_fault(std::cerr);
      exit(-1);
    }
  }
  return 0;

  //Create and call
  BaseEntity::ptr runtime = temp.Create ("runtime");
  //printf("Runtime %u\n", runtime->Get());
  Runtime::ptr rt = std::tr1::dynamic_pointer_cast<Runtime> (runtime);
  
  BaseEntity::ptr videotest = temp.Create("videotestsource");
  VideoTestSource::ptr seg = std::tr1::dynamic_pointer_cast<VideoTestSource> (videotest); 
  seg->set_runtime (rt); //..and play
  
  videotest->list_properties ();
  
  std::cout << "----- pattern  " << videotest->get_property ("pattern") << std::endl ;
  
  videotest->set_property ("pattern","snow");

  std::cout << "----- pattern  " << videotest->get_property ("pattern") << std::endl ;

  // BaseEntity::ptr myvideotest = temp.Create("videotestsource");
  // VideoTestSource::ptr myseg = std::tr1::dynamic_pointer_cast<VideoTestSource> (myvideotest); 
  // rt->add_segment (myseg);


  rt->run();
  
  return 0;
}




int switcher_controlService::add(double a, double b, double *result)
{ *result = a + b;
  return SOAP_OK;
} 

int switcher_controlService::sub(double a, double b, double *result)
{ *result = a - b;
  return SOAP_OK;
} 

int switcher_controlService::mul(double a, double b, double *result)
{ *result = a * b;
  return SOAP_OK;
} 

int switcher_controlService::div(double a, double b, double *result)
{ if (b)
    *result = a / b;
  else
  { char *s = (char*)soap_malloc(this, 1024);
    sprintf(s, "<error xmlns=\"http://tempuri.org/\">Can't divide %f by %f</error>", a, b);
    return soap_senderfault("Division by zero", s);
  }
  return SOAP_OK;
} 

int switcher_controlService::pow(double a, double b, double *result)
{ *result = ::pow(a, b);
  if (soap_errno == EDOM)	/* soap_errno is like errno, but compatible with Win32 */
  { char *s = (char*)soap_malloc(this, 1024);
    sprintf(s, "Can't take the power of %f to %f", a, b);
    sprintf(s, "<error xmlns=\"http://tempuri.org/\">Can't take power of %f to %f</error>", a, b);
    return soap_senderfault("Power function domain error", s);
  }
  return SOAP_OK;
}
