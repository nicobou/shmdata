SUBDIRS= \
    example \
    pjsip

if V4L2BUILD
SUBDIRS += \
    v4l2
endif

if OSCBUILD
SUBDIRS += \
    osc
endif

if GSOAPBUILD
SUBDIRS += \
    gsoap
endif

if GTKBUILD
SUBDIRS += \
    gtk
endif

if PULSEBUILD
SUBDIRS += \
    pulse
endif

if PORTMIDIBUILD
SUBDIRS += \
    portmidi
endif

if POSTUREBUILD
SUBDIRS += \
	posture
endif

# FIXME port to shmdata1
# if HAVE_OSX
# SUBDIRS += \
#     syphonsrc
# endif

if !HAVE_OSX
SUBDIRS += \
    systemusage
endif

if JACKBUILD
SUBDIRS += \
    jack
endif

if VNCBUILD
SUBDIRS += \
	vnc
endif
