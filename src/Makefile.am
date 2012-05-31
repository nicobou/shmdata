####### webservice code generation into the "webservices" directory
soapC.cpp: switcher-ctrl.h
	soapcpp2 -i -d webservices switcher-ctrl.h

soapswitcher_controlService.cpp: switcher-ctrl.h
	soapcpp2 -i -d webservices switcher-ctrl.h

soapswitcher_controlProxy.cpp: switcher-ctrl.h
	soapcpp2 -i -d webservices switcher-ctrl.h

noinst_PROGRAMS = \
    soapC.cpp \
    soapswitcher_controlService.cpp 

####### switcher bins
bin_PROGRAMS = \
    soapC.cpp \
    soapswitcher_controlService.cpp \
    switcher \
    switcher-ctrl

switcher_SOURCES = \
   main.cpp \
   webservices/soapC.cpp \
   webservices/soapswitcher_controlService.cpp

# noinst_HEADERS = \
#    gst-app.h \
#    play.h

switcher_CXXFLAGS = \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    -I$(top_srcdir) \
    -I$(top_srcdir)/src/webservices

switcher_LDFLAGS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

############# control ##########################

switcher_ctrl_SOURCES = \
   switcher-ctrl.cpp \
   webservices/soapC.cpp \
   webservices/soapswitcher_controlProxy.cpp

switcher_ctrl_CXXFLAGS = \
    $(GSOAP_CFLAGS) \
    -I$(top_srcdir)/src/webservices

switcher_ctrl_LDFLAGS = \
    $(GSOAP_LIBS)

clean-local:
	-rm -f \
               webservices/soapC.cpp soapH.h \
               webservices/soapStub.h \
               webservices/soapswitcher_controlProxy.cpp \
               webservices/soapswitcher_controlProxy.h \
               webservices/soapswitcher_controlService.cpp \
               webservices/soapswitcher_controlService.h \
               webservices/switcher.xsd \
               webservices/switcher_control.*.xml \
               webservices/switcher_control.nsmap \
               webservices/switcher_control.wsdl 

