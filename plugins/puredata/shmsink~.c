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

#include "m_pd.h"
#include "shmdata/any-data-writer.h"

#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ shmsink~ ----------------------------- */


static t_class *shmsink_class;

typedef struct _shmsink
{
  t_object x_obj; 	/* obligatory header */
  shmdata_any_writer_t *x_writer;
  unsigned long long x_clock;
  t_float x_f;    	/* place to hold inlet's value if it's set by message */
} t_shmsink;

    /* this is the actual performance routine which acts on the samples.
    It's called with a single pointer "w" which is our location in the
    DSP call list.  We return a new "w" which will point to the next item
    after us.  Meanwhile, w[0] is just a pointer to dsp-perform itself
    (no use to us), w[1] and w[2] are the input and output vector locations,
    and w[3] is the number of points to calculate. */
static t_int *shmsink_perform(t_int *w)
{
   t_shmsink *x = ( t_shmsink *)(w[1]);
  t_float *in = (t_float *)(w[2]);
  int n = (int)(w[3]);
  
  shmdata_any_writer_push_data (x->x_writer,
				in, 
				n*sizeof (t_float), 
				x->x_clock, 
				NULL, 
				NULL);
  
  x->x_clock += n;

    /* t_float *in = (t_float *)(w[1]); */
    /* t_float *out = (t_float *)(w[2]); */
    /* int n = (int)(w[3]); */
    /* while (n--) */
    /* { */
    /* 	float f = *(in++); */
    /* 	*out++ = (f > 0 ? f : -f); */
    /* } */
    return (w+4);
}

    /* called to start DSP.  Here we call Pd back to add our perform
    routine to a linear callback list which Pd in turn calls to grind
    out the samples. */
static void shmsink_dsp(t_shmsink *x, t_signal **sp)
{
  dsp_add(shmsink_perform, 3, x, sp[0]->s_vec, sp[0]->s_n);
}

static void *shmsink_new(void)
{
    t_shmsink *x = (t_shmsink *)pd_new(shmsink_class);
    //outlet_new(&x->x_obj, gensym("signal"));
    x->x_f = 0;
    x->x_clock = 0;
    x->x_writer = shmdata_any_writer_init ();
    /* if (verbose) */
    /*    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_ENABLE_DEBUG); */
    /*  else */
    /*    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_DISABLE_DEBUG); */
    shmdata_any_writer_set_data_type (x->x_writer, "audio/x-raw-float");
    shmdata_any_writer_set_path (x->x_writer,"/tmp/pd-shmdata-test");
    shmdata_any_writer_start (x->x_writer);
    return (x);
}

    /* this routine, which must have exactly this name (with the "~" replaced
    by "_tilde) is called when the code is first loaded, and tells Pd how
    to build the "class". */
void shmsink_tilde_setup(void)
{
    shmsink_class = class_new(gensym("shmsink~"), (t_newmethod)shmsink_new, 0,
    	sizeof(t_shmsink), 0, A_DEFFLOAT, 0);
	    /* this is magic to declare that the leftmost, "main" inlet
	    takes signals; other signal inlets are done differently... */
    CLASS_MAINSIGNALIN(shmsink_class, t_shmsink, x_f);
    	/* here we tell Pd about the "dsp" method, which is called back
	when DSP is turned on. */
    class_addmethod(shmsink_class, (t_method)shmsink_dsp, gensym("dsp"), 0);
}
