#include "m_pd.h"
#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ shmdata~ ----------------------------- */

/* tilde object to take absolute value. */

static t_class *shmdata_class;

typedef struct _shmdata
{
    t_object x_obj; 	/* obligatory header */
    t_float x_f;    	/* place to hold inlet's value if it's set by message */
} t_shmdata;

    /* this is the actual performance routine which acts on the samples.
    It's called with a single pointer "w" which is our location in the
    DSP call list.  We return a new "w" which will point to the next item
    after us.  Meanwhile, w[0] is just a pointer to dsp-perform itself
    (no use to us), w[1] and w[2] are the input and output vector locations,
    and w[3] is the number of points to calculate. */
static t_int *shmdata_perform(t_int *w)
{
    t_float *in = (t_float *)(w[1]);
    t_float *out = (t_float *)(w[2]);
    int n = (int)(w[3]);
    while (n--)
    {
    	float f = *(in++);
	*out++ = (f > 0 ? f : -f);
    }
    return (w+4);
}

    /* called to start DSP.  Here we call Pd back to add our perform
    routine to a linear callback list which Pd in turn calls to grind
    out the samples. */
static void shmdata_dsp(t_shmdata *x, t_signal **sp)
{
    dsp_add(shmdata_perform, 3, sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_n);
}

static void *shmdata_new(void)
{
    t_shmdata *x = (t_shmdata *)pd_new(shmdata_class);
    outlet_new(&x->x_obj, gensym("signal"));
    x->x_f = 0;
    return (x);
}

    /* this routine, which must have exactly this name (with the "~" replaced
    by "_tilde) is called when the code is first loaded, and tells Pd how
    to build the "class". */
void shmdata_tilde_setup(void)
{
    shmdata_class = class_new(gensym("shmdata~"), (t_newmethod)shmdata_new, 0,
    	sizeof(t_shmdata), 0, A_DEFFLOAT, 0);
	    /* this is magic to declare that the leftmost, "main" inlet
	    takes signals; other signal inlets are done differently... */
    CLASS_MAINSIGNALIN(shmdata_class, t_shmdata, x_f);
    	/* here we tell Pd about the "dsp" method, which is called back
	when DSP is turned on. */
    class_addmethod(shmdata_class, (t_method)shmdata_dsp, gensym("dsp"), 0);
}
