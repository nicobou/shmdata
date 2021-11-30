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

#ifndef __SWITCHER_LOGGER_H__
#define __SWITCHER_LOGGER_H__

/**
 * @brief Generic `SPDLOG` macros
 */

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#else
#if SPDLOG_ACTIVE_LEVEL >= SPDLOG_LEVEL_DEBUG
#undef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#endif

#include <spdlog/spdlog.h>

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
#define LOGGER_TRACE(logger, ...) SPDLOG_LOGGER_TRACE(logger, __VA_ARGS__)
#else
#define LOGGER_TRACE(logger, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG
#define LOGGER_DEBUG(logger, ...) SPDLOG_LOGGER_DEBUG(logger, __VA_ARGS__)
#else
#define LOGGER_DEBUG(logger, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_INFO
#define LOGGER_INFO(logger, ...) SPDLOG_LOGGER_INFO(logger, __VA_ARGS__)
#else
#define LOGGER_INFO(logger, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_WARN
#define LOGGER_WARN(logger, ...) SPDLOG_LOGGER_WARN(logger, __VA_ARGS__)
#else
#define LOGGER_WARN(logger, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_ERROR
#define LOGGER_ERROR(logger, ...) SPDLOG_LOGGER_ERROR(logger, __VA_ARGS__)
#else
#define LOGGER_ERROR(logger, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_CRITICAL
#define LOGGER_CRITICAL(logger, ...) SPDLOG_LOGGER_CRITICAL(logger, __VA_ARGS__)
#else
#define LOGGER_CRITICAL(logger, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_OFF
#define LOGGER_OFF(logger, ...) SPDLOG_LOGGER_OFF(logger, __VA_ARGS__)
#else
#define LOGGER_OFF(logger, ...) (void)0
#endif

#endif