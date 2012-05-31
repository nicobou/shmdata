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


//gsoap switcher service name:	    switcher-control Switcher control interface
//gsoap switcher service style:	    rpc
//gsoap switcher service encoding:  encoded
//gsoap switcher service namespace: http://localhost:8080/switcher-control.wsdl
//gsoap switcher service location:  http://localhost:8080/switcher-control.cgi

//gsoap switcher schema namespace:	urn:switcher-control

//gsoap switcher service method-documentation: add Sums two values
int switcher__add(double a, double b, double *result);

//gsoap switcher service method-documentation: sub Subtracts two values
int switcher__sub(double a, double b, double *result);

//gsoap switcher service method-documentation: mul Multiplies two values
int switcher__mul(double a, double b, double *result);

//gsoap switcher service method-documentation: div Divides two values
int switcher__div(double a, double b, double *result);

//gsoap switcher service method-documentation: pow Raises a to b
int switcher__pow(double a, double b, double *result);
