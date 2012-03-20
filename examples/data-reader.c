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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#include "shmdata/any-data-reader.h"

typedef enum _bool
{
  no = 0,
  yes = 1
} Bool;

static shmdata_any_reader_t *reader;
static Bool keep_going;

void
leave (int sig)
{
  shmdata_any_reader_close (reader);
  keep_going = no;
}

void
on_data (shmdata_any_reader_t * reader,
	 void *shmbuf,
	 void *data,
	 int data_size,
	 unsigned long long timestamp,
	 const char *type_description, void *user_data)
{
  printf ("data %p, data size %d, timestamp %llu, type descr %s\n",
	  data, data_size, timestamp, type_description);
  printf ("user_data: %s\n", (const char *) user_data);

  //free the data, can also be called later
  shmdata_any_reader_free (shmbuf);
}

int
main (int argc, char *argv[])
{
  (void) signal (SIGINT, leave);

  if (argc != 2)
    {
      printf ("Usage: %s <socket-path>\n", argv[0]);
      return -1;
    }

  const char *my_user_data = "hello world";

  reader = shmdata_any_reader_init ();
  shmdata_any_reader_set_debug (reader, SHMDATA_ENABLE_DEBUG);
  shmdata_any_reader_set_on_data_handler (reader, &on_data,
					  (void *) my_user_data);
//    shmdata_any_reader_set_data_type(reader, "video/x-raw-yuv, format=(fourcc)YUY2, framerate=(fraction)25/1, width=(int)924, height=(int)576, interlaced=(boolean)true, pixel-aspect-ratio=(fraction)1/1");
  shmdata_any_reader_set_data_type (reader, "application/helloworld_");
  shmdata_any_reader_start (reader, argv[1]);

  //shmdata_any_reader is non blocking
  keep_going = yes;
  while (keep_going == yes)
    {
      sleep (1);
    }
  return 0;
}

