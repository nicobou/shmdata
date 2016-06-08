dnl required version of autoconf
AC_PREREQ([2.53])

dnl package name and package version
AC_INIT([switcher],[0.8.10])

dnl required version of gstreamer and gst-plugins-base
GST_REQUIRED=1.0.0
GSTPB_REQUIRED=1.0.0

dnl AC_CONFIG_SRCDIR([src/main.c])
AC_CONFIG_HEADERS([config.h])

dnl required version of automake
dnl 'foreign' allows for accepting README.md instead of README 
AM_INIT_AUTOMAKE([foreign])  

dnl enable mainainer mode by default
dnl AM_MAINTAINER_MODE([enable])

dnl sets host_* variables
AC_CANONICAL_HOST

# Define these substitions here to keep all version information in one place.
# For information on how to properly maintain the library version information,
# refer to the libtool manual, section "Updating library version information":
# http://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
# see http://sourceware.org/autobook/autobook/autobook_91.html
# format is [current:revision:age]
# libtool versioning is different than the project's versioning. It's for the ABI and API.
# It's in the form "current:revision:age" where:
#  * current: Increment when the interface changes. (and set revision to 0)
#  * revision: Increment when the implementation changes, but not the interface.
#  * age: Backward-compatible with how many interface numbers.
AC_SUBST([LIBSWITCHER_SO_VERSION], [0:0:0])

# The version of the API. Should be major.minor and not change until not backward-compatible
AC_SUBST([LIBSWITCHER_API_VERSION], [0.8])
AC_DEFINE(LIBSWITCHER_API_VERSION, ["0.8"], [API version])

dnl check for tools (compiler etc.)
dnl g++-4.9 is here in order to get threaded-wrapper compiling on ubuntu 14.04
AC_PROG_CXX([g++-4.9])
AM_PROG_CC_C_O
AC_PROG_INSTALL
AC_PROG_LIBTOOL

dnl required version of libtool
LT_PREREQ([2.2.6])
LT_INIT


dnl error out if we can't find pkg-config
AC_CHECK_PROG(HAVE_PKGCONFIG, pkg-config, [ ], [
  AC_MSG_ERROR([You need to have pkg-config installed or set the PATH.])
])

## for building syphon, but seems it must be call unconditionally 
AC_PROG_OBJCXX
case "$host" in *-darwin* | *-macos10*)
  LIBTOOL="glibtool"
  CXXFLAGS+=" -D__Darwin -stdlib=libc++ -Qunused-arguments"
  CXX="clang++"
  LDFLAGS+=" -g -undefined dynamic_lookup -Wl,-headerpad_max_install_names"
  if test -d /opt/local ; then
    CPPFLAGS+=" -I/usr/local/include -I/opt/local/include"
    CXXFLAGS+=" -I/usr/local/include -I/opt/local/include"
    LDFLAGS+=" -L/usr/local/lib -L/opt/local/lib"
  elif test -d /sw ; then
    CPPFLAGS+=" -I/sw/include"
    CXXFLAGS+=" -I/sw/include"
    LDFLAGS+=" -L/sw/lib"
  fi
  
  have_osx=true;
  AM_CONDITIONAL([HAVE_OSX], test "true" = "true")
  AC_DEFINE(HAVE_OSX,[1],[Apple Mac OS X operating system detected])
  
  # OSX does not include /usr/local/lib/pkgconfig path, so we export it here
  # (for everyone, just in case)
  PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
  export PKG_CONFIG_PATH
  
  LDFLAGS+=" -fPIC -L/usr/local/lib"
  ;;
*)
  have_osx=false;
  AM_CONDITIONAL([HAVE_OSX], test "true" = "false")
  AC_DEFINE(HAVE_OSX,[0],[Apple Mac OS X operating system not detected])
  ;;
esac

dnl =======================================================================
dnl Check for the required version of GStreamer core (and gst-plugins-base)
dnl
dnl This will export GST_CFLAGS and GST_LIBS variables for use in Makefile.am
dnl

PKG_CHECK_MODULES(GST, [
    gstreamer-1.0              >= $GST_REQUIRED
    gstreamer-base-1.0         >= $GST_REQUIRED
    gstreamer-controller-1.0   >= $GST_REQUIRED
    gstreamer-sdp-1.0          >= $GST_REQUIRED
    gstreamer-video-1.0        >= $GST_REQUIRED
  ], [
    AC_SUBST(GST_CFLAGS)
    AC_SUBST(GST_LIBS)
  ], [
  AC_MSG_ERROR([
    Can't find the following GStreamer development packages:

      gstreamer-1.0              >= $GST_REQUIRED
      gstreamer-base-1.0         >= $GST_REQUIRED
      gstreamer-controller-1.0   >= $GST_REQUIRED
      gstreamer-sdp-1.0          >= $GST_REQUIRED
      gstreamer-video-1.0        >= $GST_REQUIRED

    You need to install or upgrade the GStreamer development
      packages on your system. On debian-based systems these are
      libgstreamer1.0-dev and libgstreamer-plugins-base1.0-dev.
      on RPM-based systems gstreamer1.0-devel, libgstreamer1.0-devel
      or similar. The minimum version required is $GST_REQUIRED.
  ])
])

PKG_CHECK_MODULES(GLIB, glib-2.0, have_glib=true, have_glib=false)
if test "x${have_glib}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  libglib2.0-dev  ])
fi

PKG_CHECK_MODULES(GMODULE, gmodule-2.0, have_gmodule=true, have_gmodule=false)
if test "x${have_gmodule}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  libglib2.0-dev  ])
fi

#shmdata
PKG_CHECK_MODULES(SHMDATA, shmdata-1.2, have_shmdata=true, have_shmdata=false)
if test "x${have_shmdata}" = "xfalse" ; then
    AC_MSG_ERROR([Missing libshmdata-1.2 development files])
fi

#gsoap
PKG_CHECK_MODULES(GSOAP, gsoap++, have_gsoap=true, have_gsoap=false)
if test "x${have_gsoap}" = "xfalse" ; then
    GSOAPBUILD=disabled
    AC_MSG_WARN([Missing gsoap package])
else
    GSOAPBUILD=enabled
fi
AM_CONDITIONAL(GSOAPBUILD, test "${GSOAPBUILD}" = "enabled")

#liblo
PKG_CHECK_MODULES(LIBLO, liblo, have_liblo=true, have_liblo=false)
if test "x${have_liblo}" = "xfalse" ; then
    OSCBUILD=disabled
    AC_MSG_WARN([Missing liblo-dev package, switcher will not be built])
else
    OSCBUILD=enabled
fi
AM_CONDITIONAL(OSCBUILD, test "${OSCBUILD}" = "enabled")

#jack
PKG_CHECK_MODULES(JACK, jack, have_jack=true, have_jack=false)
if test "x${have_jack}" = "xfalse" ; then
    JACKBUILD=disabled
    AC_MSG_WARN([Missing libjack-jackd2-dev package])
else
    JACKBUILD=enabled
fi
AM_CONDITIONAL(JACKBUILD, test "${JACKBUILD}" = "enabled")

# python
PKG_CHECK_MODULES(PYTHON, python, have_python=true, have_python=false)
if test "x${have_python}" = "xfalse" ; then
    PYTHONBUILD=disabled
    AC_MSG_WARN([Missing python-dev package])
    AC_DEFINE(HAVE_PYTHON,[0],[python-dev not detected])
else
    PYTHONBUILD=enabled
    AC_DEFINE(HAVE_PYTHON,[1],[python-dev detected])
fi
AM_CONDITIONAL(PYTHONBUILD, test "${PYTHONBUILD}" = "enabled")

# portmidi
PORTMIDIBUILD=enabled
AC_CHECK_HEADERS([portmidi.h porttime.h], have_portmidi_headers=true, have_portmidi_headers=false)
if test "x${have_portmidi_headers}" = "xfalse" ; then
   PORTMIDIBUILD=disabled
   AC_MSG_WARN([missing portmidi headers, install libportmidi-dev])
fi
AC_CHECK_LIB([portmidi],[Pm_Initialize], have_portmidi=true, have_portmidi=false)
if test "x${have_portmidi}" = "xfalse" ; then
   PORTMIDIBUILD=disabled
   AC_MSG_WARN([missing portmidi library, install libportmidi-dev])
fi
dnl AC_CHECK_LIB([porttime],[Pt_Time], have_porttime=true, have_porttime=false)
dnl if test "x${have_porttime}" = "xfalse" ; then
dnl    PORTMIDIBUILD=disabled
dnl    AC_MSG_WARN([missing porttime library, install libportmidi-dev])
dnl fi
AM_CONDITIONAL(PORTMIDIBUILD, test "${PORTMIDIBUILD}" = "enabled")

if test "${PORTMIDIBUILD}" = "enabled"; then
  PORTMIDI_CFLAGS="-DPMALSA"
  AC_SUBST([PORTMIDI_CFLAGS], "$PORTMIDI_CFLAGS")
  dnl PORTMIDI_LIBS="-lportmidi -lporttime -lpthread -lm"
  PORTMIDI_LIBS="-lportmidi -lpthread -lm"
  AC_SUBST([PORTMIDI_LIBS], "$PORTMIDI_LIBS")
fi

#pulse
PKG_CHECK_MODULES(PULSE, libpulse-mainloop-glib, have_libpulse=true, have_libpulse=false)
if test "x${have_libpulse}" = "xfalse" ; then
    PULSEBUILD=disabled
    AC_MSG_WARN([Missing libpulse-dev])
else
    PULSEBUILD=enabled
fi
AM_CONDITIONAL(PULSEBUILD, test "${PULSEBUILD}" = "enabled")

#json
PKG_CHECK_MODULES(JSONGLIB, json-glib-1.0, have_jsonglib=true, have_jsonglib=false)
if test "x${have_jsonglib}" = "xfalse" ; then
    AC_MSG_ERROR([Missing libjson-glib-dev package])
fi

# check for v4l2 
AC_CHECK_HEADER(linux/videodev2.h,
                V4L2BUILD=enabled,
                [AC_MSG_WARN([v4l2 capture plugin will not be built (missing videodev2.h from package linux-libc-dev)])
                V4L2BUILD=disabled])
AM_CONDITIONAL(V4L2BUILD, test "${V4L2BUILD}" = "enabled")

#pjsip
dnl PKG_CHECK_MODULES(PJSIP, libpjproject, have_pjsip=true, have_pjsip=false)
dnl if test "x${have_pjsip}" = "xfalse" ; then
dnl     PJSIPBUILD=disabled
dnl     AC_MSG_WARN([Missing pjsip dev files])
dnl else
dnl     PJSIPBUILD=enabled
dnl fi
dnl AM_CONDITIONAL(PJSIPBUILD, test "${PJSIPBUILD}" = "enabled")


# check for gtk (compatible with gstreamer sdk)
PKG_CHECK_MODULES(GTK, gtk+-2.0, have_gtk=true, have_gtk=false)
if test "x${have_gtk}" = "xfalse" ; then
    GTKBUILD=disabled
    AC_MSG_WARN([Missing libgtk2.0-dev package, gtk fullscreen video will not be built])
else
    GTKBUILD=enabled
fi

if test "x${have_osx}" = "xtrue"; then
    GTKBUILD=disabled
    AC_MSG_WARN([gtkwin is not going to be built on OSX])
fi

AM_CONDITIONAL(GTKBUILD, test "${GTKBUILD}" = "enabled")
if test "${GTKBUILD}" = "enabled"; then
    AC_DEFINE(HAVE_GTK, [1], [GTK enabled])
else
    AC_DEFINE(HAVE_GTK, [0], [GTK disabled])
fi

# check for posturevision
PKG_CHECK_MODULES(POSTURE, posture, have_posture=true, have_posture=false)
if test "x${have_posture}" = "xfalse" ; then
    POSTUREBUILD=disabled
    AC_MSG_WARN([Missing Posturevision, there won't be support for point clouds captures])
else
    POSTUREBUILD=enabled
fi
AM_CONDITIONAL(POSTUREBUILD, test "${POSTUREBUILD}" = "enabled")

# check for libvncclient
PKG_CHECK_MODULES(VNC, libvncclient, have_vnc=true, have_vnc=false)
if test "x${have_vnc}" = "xfalse" ; then
    VNCBUILD=disabled
    AC_MSG_WARN([Missing libvncclient, VNC client quiddity won't be built])
else
    VNCBUILD=enabled
fi
AM_CONDITIONAL(VNCBUILD, test "${VNCBUILD}" = "enabled")


dnl Use the C++ compiler for the compile tests
AC_LANG(C++)

AC_MSG_CHECKING([to see if compiler understands -std=c++11])
CXXFLAGS="$CXXFLAGS -std=c++11"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  AC_MSG_RESULT([yes])
], [
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([compiler does not understand -std=c++11])
])

AC_MSG_CHECKING([to see if compiler can expand variadic template in lambda])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[void do_nothing(int, double){} template<typename ...Args> void do_lambda(Args... args){ [&](){ do_nothing(args...); }(); } ]], 
[[ do_lambda(1, 2.0); ]])], [
  AC_MSG_RESULT([yes])
], [
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([
     compiler cannot expand variadic template in lambda (require g++ >= 4.9)
     on Ubuntu 14.04, you can install g++-4.9 doing the following:
     sudo add-apt-repository ppa:ubuntu-toolchain-r/test
     sudo apt-get update
     sudo apt-get install g++-4.9
])
])


dnl check if compiler understands -Wall (if yes, add -Wall to CFLAGS)
AC_MSG_CHECKING([to see if compiler understands -Wall])
saved_CXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS -Wall"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  AC_MSG_RESULT([yes])
], [
  CXXFLAGS="$saved_CXXFLAGS"
  AC_MSG_RESULT([no])
])

dnl check if compiler understands -Wextra (if yes, add -Wextra to CFLAGS)
AC_MSG_CHECKING([to see if compiler understands -Wextra])
saved_CXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS -Wextra"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  AC_MSG_RESULT([yes])
], [
  CXXFLAGS="$saved_CXXFLAGS"
  AC_MSG_RESULT([no])
])
          
dnl check if compiler understands -Weffc++ (if yes, add -Weffc++ to CFLAGS)
dnl AC_MSG_CHECKING([to see if compiler understands -Weffc++])
dnl saved_CXXFLAGS="$CXXFLAGS"
dnl CXXFLAGS="$CXXFLAGS -Weffc++ -Wno-error=effc++"
dnl AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
dnl   AC_MSG_RESULT([yes])
dnl ], [
dnl   CXXFLAGS="$saved_CXXFLAGS"
dnl   AC_MSG_RESULT([no])
dnl ])

dnl check if compiler understands -Werror (if yes, add it to CFLAGS)
dnl -Wno-error=unused-parameter is required for gsoap generated code compiling
AC_MSG_CHECKING([to see if compiler understands -Werror])
saved_CXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS -Werror -Wno-error=unused-parameter"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  CXXFLAGS="$saved_CXXFLAGS -Werror -Wno-error=unused-parameter"
  AC_SUBST([W_NO_ERROR_UNUSED_PARAMETER], "-Wno-error=unused-parameter")
  AC_MSG_RESULT([yes])
], [
  CXXFLAGS="$saved_CXXFLAGS"
  AC_MSG_RESULT([no])
])

dnl clang++
dnl check if compiler understands -Wno-error=return-type-c-linkage 
dnl (if yes, add it to CFLAGS)
dnl -Wno-error=return-typer-c-linkage is required for gsoap generated code compiling
AC_MSG_CHECKING([to see if compiler understands -Wno-error=return-type-c-linkage])
saved_CXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS -Wno-error=return-type-c-linkage"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  CXXFLAGS="$saved_CXXFLAGS -Wno-error=return-type-c-linkage"
  AC_MSG_RESULT([yes])
], [
  CXXFLAGS="$saved_CXXFLAGS"
  AC_MSG_RESULT([no])
])

dnl clang++
dnl check if compiler understands -Wno-error=extra 
dnl (if yes, add it to CFLAGS)
dnl -Wno-error=extra is required for gsoap generated code compiling
AC_MSG_CHECKING([to see if compiler understands -Wno-error=extra])
saved_CXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS -Wno-error=extra"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  CXXFLAGS="$saved_CXXFLAGS -Wno-error=extra"
  AC_SUBST([W_NO_ERROR_EXTRA], "-Wno-error=extra")
  AC_MSG_RESULT([yes])
], [
  CXXFLAGS="$saved_CXXFLAGS"
  AC_MSG_RESULT([no])
])

dnl clang++
dnl check if compiler understands -Wno-error=deprecated-register
dnl (if yes, add it to CFLAGS)
dnl required for gstreamer header inclusion
AC_MSG_CHECKING([to see if compiler understands -Wno-error=deprecated-register])
saved_CXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS -Wno-error=deprecated-register"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  CXXFLAGS="$saved_CXXFLAGS -Wno-error=deprecated-register"
  AC_MSG_RESULT([yes])
], [
  CXXFLAGS="$saved_CXXFLAGS"
  AC_MSG_RESULT([no])
])
                                          
AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test -z "$DOXYGEN";
    then AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test -n "$DOXYGEN"])

dnl testing if switcher can be built and add it to CONFIG_FILES if building
SWITCHERBUILD=enabled
if test "${OSCBUILD}" = "disabled"; 
  then SWITCHERBUILD=disabled
fi
if test "${SOAPBUILD}" = "disabled";
  then SWITCHERBUILD=disabled
fi
AM_CONDITIONAL(SWITCHERBUILD, test "${SWITCHERBUILD}" = "enabled")

dnl syphon
SYPHONBUILD=disabled
if test "x${have_osx}" = "xtrue"; then
    SYPHONBUILD=enabled
fi
AM_CONDITIONAL(SYPHONBUILD, test "${SYPHONBUILD}" = "enabled")

dnl nvenc
NVENCBUILD=disabled

PKG_CHECK_MODULES([CUDA], [cuda-7.5 cudart-7.5],, [
      PKG_CHECK_MODULES([CUDA], [cuda-7.0 cudart-7.0],, [
        PKG_CHECK_MODULES([CUDA], [cuda-6.5 cudart-6.5],, [
          AC_MSG_WARN([Could not find cuda headers/libraries (nvenc plugin)])])])])
HAVE_CUDA_H=no
HAVE_CUDART_H=no
save_CPPFLAGS="$CPPFLAGS"
CPPFLAGS="$CUDA_CFLAGS $save_CPPFLAGS "
AC_CHECK_HEADER([cuda.h], [HAVE_CUDA_H=yes],
    AC_MSG_WARN([Could not find cuda.h  (nvenc plugin)]))
AC_CHECK_HEADER([cuda_runtime_api.h], [HAVE_CUDART_H=yes],
    AC_MSG_WARN([Could not find cuda_runtime_api.h  (nvenc plugin)]))
CPPFLAGS=$save_CPPFLAGS
save_LIBS="$LIBS"
LIBS="$CUDA_LIBS $save_LIBS"
HAVE_CUDART_LIB="no"
AC_CHECK_LIB(cudart,cudaGetErrorString,[HAVE_CUDART_LIB="yes"], [
    AC_MSG_WARN([Could not find cudart library  (nvenc plugin)])])
HAVE_CUDA_LIB="no"
AC_CHECK_LIB(cuda,cuInit,[HAVE_CUDA_LIB="yes"], [
    AC_MSG_WARN([Could not find cuda library  (nvenc plugin)])])

HAVE_NVIDIA_ENCODE_LIB="no"
AC_ARG_VAR(NVENC_LIBS, [linker flags for nvidia-encode])
LIBS="$NVENC_LIBS $save_LIBS"
AC_CHECK_LIB(nvidia-encode,NvEncodeAPICreateInstance,[HAVE_NVIDIA_ENCODE_LIB="yes"], [
    AC_MSG_WARN([
    -----------------------
    Could not find nvidia-encode library (nvenc plugin), try configure with
    NVENC_LIBS='-L/usr/lib/nvidia-<driver-version-number>' ./configure
    -----------------------
])])
NVENCODE_LIBS="$NVENCODE_LIBS -lnvidia-encode"
AC_SUBST(NVENCODE_LIBS)
LIBS="$saved_LIBS"

HAVE_NVENC_SDK_PATH="no"
save_CPPFLAGS="$CPPFLAGS"
AC_ARG_VAR(NVENC_SDK_PATH, [path to nvenc 5.0.1 SDK])
CPPFLAGS="-I$NVENC_SDK_PATH $CPPFLAGS"
AC_CHECK_HEADER(Samples/common/inc/nvEncodeAPI.h,
                [HAVE_NVENC_SDK_PATH="yes"],
                [AC_MSG_WARN([
    -----------------------
    Could not find header from nvenc SDK version 5.0.1, try configure with
    NVENC_SDK_PATH='<path-to-nvenc-sdk>' NVENC_LIBS='-L/usr/lib/nvidia-<driver-version-number>' ./configure
    -----------------------])],
                [])
AC_SUBST(NVENC_SDK_PATH)
CPPFLAGS="$save_CPPFLAGS"


if test "x$HAVE_CUDA_H" = "xyes" \
    -a "x$HAVE_CUDART_H" = "xyes" \
    -a "x$HAVE_CUDA_LIB" = "xyes" \
    -a "x$HAVE_NVIDIA_ENCODE_LIB" = "xyes" \
    -a "x$HAVE_NVENC_SDK_PATH" = "xyes" \
    -a "x$HAVE_CUDART_LIB" = "xyes"; then
  NVENCBUILD="enabled"
fi
AM_CONDITIONAL(NVENCBUILD, test "x${NVENCBUILD}" = "xenabled")

dnl =======================================================================
dnl Finally, create Makefiles in all directories

AC_CONFIG_FILES([
Makefile
doxyfile
switcher-${LIBSWITCHER_API_VERSION}.pc:switcher.pc.in
switcher/Makefile
tests/Makefile
src/Makefile
plugins/Makefile
plugins/example/Makefile
plugins/gsoap/Makefile
plugins/gtk/Makefile
plugins/jack/Makefile
plugins/nvenc/Makefile
plugins/osc/Makefile
plugins/pjsip/Makefile
plugins/portmidi/Makefile
plugins/posture/Makefile
plugins/pulse/Makefile
plugins/syphonsrc/Makefile
plugins/systemusage/Makefile
plugins/v4l2/Makefile
plugins/vnc/Makefile
])
AC_OUTPUT

[echo ""]
[echo "-----------------------------------"]
[echo ""]
[echo "OPTIONAL PLUGINS"]
[echo ""]
[echo "V4L2 capture               ${V4L2BUILD}"]
[echo "GTK fullscreen video       ${GTKBUILD}"]
[echo "Pulse audio                ${PULSEBUILD}"]
[echo "PortMidi                   ${PORTMIDIBUILD}"]
[echo "Jack                       ${JACKBUILD}"]
[echo "nvenc                      ${NVENCBUILD}"]
[echo "syphon reader              ${SYPHONBUILD}"]
[echo ""]
[echo "-----------------------------------"]
[echo ""]
[echo "SWITCHER REQUIRED PLUGINS"]
[echo ""]
[echo "OSC controler plugin       ${OSCBUILD}"]
[echo "GSOAP controler plugin     ${GSOAPBUILD}"]
[echo ""]
[echo "-----------------------------------"]
[echo ""]
[echo "SWITCHER OPTIONAL FEATURE"]
[echo ""]
[echo "python                     ${PYTHONBUILD}"]
[echo "posture support            ${POSTUREBUILD}"]
[echo "vnc client support         ${VNCBUILD}"]
[echo ""]
[echo "-----------------------------------"]
[echo ""]
[echo "SWITCHER BUILD STATUS"]
[echo ""]
[echo "libswitcher                enabled"]
[echo "switcher                   ${SWITCHERBUILD}"]
[echo ""]
[echo "-----------------------------------"]
[echo ""]

