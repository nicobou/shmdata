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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

#include <unistd.h>		//sleep
#include "shmdata/any-data-writer.h"

shmdata_any_writer_t *writer;

static void data_not_required_anymore (void *priv);

//clean up pipeline when ctrl-c
void
leave (int sig)
{
  shmdata_any_writer_close (writer);
  exit (sig);
}

int
main (int argc, char *argv[])
{
  (void) signal (SIGINT, leave);

  /* Check input arguments */
  if (argc != 2)
    {
      printf ("Usage: %s <socket-path>\n", argv[0]);
      return -1;
    }

  writer = shmdata_any_writer_init ();
  if (! shmdata_any_writer_set_path (writer, argv[1]))
    {
      g_printerr ("**** The file %s exists, therefore a shmdata cannot be operated with this path.\n",argv[1]);
      shmdata_any_writer_close (writer);
      exit(0);
    }
  shmdata_any_writer_set_debug (writer, SHMDATA_ENABLE_DEBUG);
  shmdata_any_writer_set_data_type (writer, "application/helloworld_");
  shmdata_any_writer_start (writer);

  unsigned long long myclock = 0;
  unsigned long long nsecPeriod = 30000000;

  char hello[21] = "helloworldhelloworld";

  while (0 == 0)
    {
      //data should be serialized if network is involved
      shmdata_any_writer_push_data (writer,
				    hello,
				    sizeof (hello),
				    myclock,
				    &data_not_required_anymore, hello);
      usleep (nsecPeriod / 1000);
      myclock += nsecPeriod;
    }

  return 0;
}

static void
data_not_required_anymore (void *priv)
{
  //printf ("freeing buffer for pointer %p\n", priv);
  //free (priv);
}

