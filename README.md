             __            __     __      
        ___ / /  __ _  ___/ /__ _/ /____ _
       (_-</ _ \/  ' \/ _  / _ `/ __/ _ `/
      /___/_//_/_/_/_/\_,_/\_,_/\__/\_,_/ 


# shmdata
Library to share flows of data frames between processes via shared memory. 
shmdata can be compared to the JACK audio connection kit project or to the
VideoJack project. shmdata, however, does not provide a host server and require applications to link data streams using socket path (e.g. "/tmp/my_shmdata_stream"). 
Note that shmdata followers and writers can hot connect/disconnect/reconnect/... 
and that a shared memory supports a single writer with multiple readers.

shmdata is currently used in several metalab projects:

* switcher (http://code.sat.qc.ca/redmine/projects/switcher/wiki)
* splash (http://code.sat.qc.ca/redmine/projects/splash)
* posturevision (http://code.sat.qc.ca/redmine/projects/kinectvision/wiki/)
* scenic (http://code.sat.qc.ca/redmine/projects/scenic/wiki)

License: LGPL

Project URL: http://code.sat.qc.ca/redmine/projects/libshmdata

# Installation
Here is how to build and install it on Debian GNU/Linux or Ubuntu::

    $ sudo apt install cmake bison build-essential flex libtool
    $ # this is only for building gstreamer plugins:
    $ sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    $ # this is for building python wrappers 
    $ sudo apt-get install python3-dev
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install
    $ sudo ldconfig
  
  
## Other build options

* To run the tests

        make test
    
* To generate installation packages (as configured in CMakeLists.txt)

        make package
        
* To generate a source package

        make package_source
        
* To test the source package, this will create the source package and then try to build and test it

        make package_source_test
        
    

# Mac OS Notes
* If you are using homebrew to install dependencies and encountering errors about ```-lintl```, you have to ```brew link gettext```
* If you are using python3 from homebrew, cmake will probably not detect it

# Authors
see git history

