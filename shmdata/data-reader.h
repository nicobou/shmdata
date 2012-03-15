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

#ifndef _SHMDATA_DATA_READER_H_
#define _SHMDATA_DATA_READER_H_


#ifdef __cplusplus
 extern "C" {
#endif
     
#define SHMDATA_DATA_READER_ENABLE_DEBUG 1
#define SHMDATA_DATA_READER_DISABLE_DEBUG 0

typedef struct shmdata_data_reader_ shmdata_data_reader_t;
     
shmdata_data_reader_t *shmdata_data_reader_init ();
     
void shmdata_data_reader_set_debug (shmdata_data_reader_t *context,
				    int debug);

void shmdata_data_reader_set_on_data_handler (shmdata_data_reader_t *context,
					      void(*on_data)(shmdata_data_reader_t *,
							     void *, 
							     int, 
							     unsigned long long, 
							     const char *,
							     void *), 
					      void *user_data);

//if not called, any type of data is forwarded to the on_first_data handler
//type can be either a gstreamer caps (gst_caps_to_string) or a user defined string 
void shmdata_data_reader_set_data_type(shmdata_data_reader_t *context,
				       const char *type);
     
void shmdata_data_reader_start (shmdata_data_reader_t *context,
				const char *socketName);

void shmdata_data_reader_close(shmdata_data_reader_t *context);
     
#ifdef __cplusplus
 }
#endif /* extern "C" */
#endif //_SHMDATA_DATA_READER_H_
