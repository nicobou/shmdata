/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#ifndef __SHMDATA_EXCEPTIONS_H__
#define __SHMDATA_EXCEPTIONS_H__

#include <stdexcept>
#include <string>

namespace shmdata
{

class BaseException : public std::runtime_error
{
    public:
        BaseException(const char *error_message) :
            std::runtime_error(error_message)
        {}
        // BaseException(const std::string &error_message) :
        //     std::runtime_error(error_message.c_str())
        // {}
        BaseException() :
            std::runtime_error("")
        {}
};

} // end of namespace

#endif // ifndef
