all: installed
  
FILENAME = osgWorks_02_00_00.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL= http://osgworks.googlecode.com/files/$(FILENAME)
SOURCE_DIR = build/osgw2
UNPACK_CMD = tar xvf
MD5SUM_FILE = $(FILENAME).md5sum
TARBALL_PATCH = osgWorks_isTransparent.patch
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = `rospack find osgWorks`/
CMAKE = cmake
CMAKE_ARGS= -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR)

installed: wiped $(SOURCE_DIR)/unpacked 
	cd $(SOURCE_DIR)
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed


docs: 
#	cd $(SOURCE_DIR)/build && make docs
#	ln -s $(SOURCE_DIR)/build/doc doc

wiped: Makefile $(FILENAME).md5sum
	make wipe
	touch wiped

clean:
	rm -rf lib
	rm -rf include
	rm -rf share
	rm -rf build
	rm -rf installed

wipe: 	clean
	touch wiped
