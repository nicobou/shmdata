LIB_SHM_DATA_SRC_FILES = shmdata-writer.cpp shmdata-reader.cpp
LIB_SHM_DATA_O_FILES = $(LIB_SHM_DATA_SRC_FILES:%.cpp=%.o)

DEPS=`pkg-config --cflags --libs gstreamer-0.10 gio-2.0`

all: libshmdata.so video_writer video_reader clutter_reader osg_reader data_writer data_reader data_reader_to_rtp data_reader_from_rtp data_writer_from_rtp pcd_writer pcd_reader

libshmdata.so: $(LIB_SHM_DATA_O_FILES)
	rm -f $@
	g++ -shared $(LIB_SHM_DATA_O_FILES) -o $@

%.o: %.cpp shmdata.h
	g++ -fPIC -c -Wall $< -o $@ $(DEPS)

video_writer: libshmdata.so video-writer.cpp
	g++ -Wall video-writer.cpp -o video_writer -L. -lshmdata $(DEPS)

video_reader: libshmdata.so video-reader.cpp
	g++ -Wall video-reader.cpp -o video_reader -L. -lshmdata $(DEPS)

clutter_reader: libshmdata.so clutter-actor-reader.cpp shmdata.h
	g++ -Wall clutter-actor-reader.cpp -o clutter_actor_reader -L. -lshmdata `pkg-config --cflags --libs gstreamer-0.10 clutter-1.0 glew`

osg_reader: libshmdata.so osg-reader.cpp shmdata.h
	g++ -Wall osg-reader.cpp -o osg_reader -L. -lshmdata `pkg-config --cflags --libs gstreamer-0.10 openscenegraph gio-2.0 gstreamer-app-0.10`

data_writer: libshmdata.so data-writer.cpp shmdata.h
	g++ -Wall data-writer.cpp -o data_writer -L. -lshmdata `pkg-config --cflags --libs gstreamer-0.10 gio-2.0 gstreamer-app-0.10`

data_reader: libshmdata.so data-reader.cpp shmdata.h
	g++ -Wall data-reader.cpp -o data_reader -L. -lshmdata `pkg-config --cflags --libs gstreamer-0.10 gio-2.0 gstreamer-app-0.10`

data_reader_to_rtp: libshmdata.so data-reader-to-rtp.cpp
	g++ -Wall data-reader-to-rtp.cpp -o data_reader_to_rtp -L. -lshmdata `pkg-config --cflags --libs gstreamer-0.10 gio-2.0 gstreamer-app-0.10`

data_writer_from_rtp: libshmdata.so data-writer-from-rtp.cpp
	g++ -Wall data-writer-from-rtp.cpp -o data_writer_from_rtp -L. -lshmdata $(DEPS)

data_reader_from_rtp: libshmdata.so data-reader-from-rtp.cpp
	g++ -Wall data-reader-from-rtp.cpp -o data_reader_from_rtp -L. -lshmdata `pkg-config --cflags --libs gstreamer-0.10 gio-2.0 gstreamer-app-0.10`

pcd_writer: libshmdata.so pcd_writer.cpp
	g++ -o pcd_writer pcd_writer.cpp -L. -lshmdata `pkg-config --cflags --libs pcl_visualization-1.4 gstreamer-0.10 gio-2.0 gstreamer-app-0.10` -I /usr/include/vtk-5.6/

pcd_reader: libshmdata.so pcd_reader.cpp
	g++ -o pcd_reader pcd_reader.cpp -L. -lshmdata `pkg-config --cflags --libs pcl_visualization-1.4 gstreamer-0.10 gio-2.0 gstreamer-app-0.10` -I /usr/include/vtk-5.6/

clean:
	rm -f *.o *.so video_writer video_reader osg_reader clutter_actor_reader data_writer data_reader *~ data_reader_to_rtp data_reader_from_rtp data_writer_from_rtp pcd_reader pcd_writer
