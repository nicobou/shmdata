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

#include "soapswitcher_controlProxy.h"
#include "switcher_control.nsmap"

const char server[] = "http://localhost:8080/switcher_controlserver.cgi";

int main(int argc, char **argv)
{ if (argc < 4)
  { fprintf(stderr, "Usage: [add|sub|mul|div|pow] num num\n");
    exit(0);
  }
  double a, b, result;
  a = strtod(argv[2], NULL);
  b = strtod(argv[3], NULL);
  switcher_controlProxy switcher_control;
  switcher_control.soap_endpoint = server;
  switch (*argv[1])
  { case 'a':
      switcher_control.add(a, b, &result);
      break;
    case 's':
      switcher_control.sub(a, b, &result);
      break;
    case 'm':
      switcher_control.mul(a, b, &result);
      break;
    case 'd':
      switcher_control.div(a, b, &result);
      break;
    case 'p':
      switcher_control.pow(a, b, &result);
      break;
    default:
      fprintf(stderr, "Unknown command\n");
      exit(0);
  }
  if (switcher_control.error)
    switcher_control.soap_stream_fault(std::cerr);
  else
    printf("result = %g\n", result);
  return 0;
}

