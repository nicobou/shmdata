SUBDIRS= \
    example

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

if PJSIPBUILD
SUBDIRS += \
    pjsip
endif

if POSTUREBUILD
SUBDIRS += \
	posture
endif

if HAVE_OSX
SUBDIRS += \
    syphonsrc
endif

if !HAVE_OSX
SUBDIRS += \
    systemusage
endif

#FIXME this will remove from sources what cannot be compiled 
#in the machine where make dist has been asked
DIST_SUBDIRS = ${SUBDIRS}
