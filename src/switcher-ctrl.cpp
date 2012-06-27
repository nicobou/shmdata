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

#include "switcher/webservices/soapcontrolProxy.h"
#include "switcher/webservices/control.nsmap"

const char server[] = "http://localhost:8080";

int main(int argc, char **argv)
{ 


  if (argc < 4)
    { fprintf(stderr, "Usage: [list|base-entities|set-prop|get-prop|add|sub|mul|div|pow] num num\n");
         exit(0);
    }
  double a, b, result;
  std::vector<std::string> resultlist;
  std::string val;


  a = strtod(argv[2], NULL);
  b = strtod(argv[3], NULL);

  controlProxy switcher_control;
  switcher_control.soap_endpoint = server;
  
  int i;
  switch (*argv[1])
    { case 'a':
	switcher_control.add(a, b, &result);
	printf("result = %g\n", result);
	break;
    // case 'sub':
    //   switcher_control.sub(a, b, &result);
    //   printf("result = %g\n", result);
    //   break;
    case 'm':
      switcher_control.mul(a, b, &result);
      printf("result = %g\n", result);
      break;
    case 'd':
      switcher_control.div(a, b, &result);
      printf("result = %g\n", result);
      break;
    case 'p':
      switcher_control.pow(a, b, &result);
      printf("result = %g\n", result);
      break;
    case 'l':
      switcher_control.list_factory_capabilities(&resultlist);
      for(i=0; i < resultlist.size(); i++)
	{
	  std::cout << resultlist[i] << std::endl;
	}
      break;
    case 'b':
      switcher_control.list_base_entities(&resultlist);
      for(i=0; i < resultlist.size(); i++)
	{
	  std::cout << resultlist[i] << std::endl;
	}
      break;
    case 's':
      switcher_control.set_entity_property (argv[2], argv[3], argv[4]);
      break;
    case 'g':
      switcher_control.get_entity_property(argv[2], argv[3],&val);
      std::cout << "val is"<< val << std::endl;
      break;
     
    default:
      fprintf(stderr, "Unknown command\n");
      exit(0);
    }

  if (switcher_control.error)
    switcher_control.soap_stream_fault(std::cerr);

    return 0;
}

