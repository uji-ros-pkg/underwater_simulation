all: installed
  
FILENAME = osgOcean-Source-1.0.1.rar
TARBALL = build/$(FILENAME)
TARBALL_URL= http://osgocean.googlecode.com/files/$(FILENAME)
TARBALL_PATCH= osgOcean-1.0.1.patch
SOURCE_DIR = build/osgOcean
UNPACK_CMD = unrar x
MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = `rospack find osgOcean`/
CMAKE = cmake
CMAKE_ARGS= -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DUSE_FFTSS=OFF -DUSE_FFTW3F=ON /

installed: wiped $(SOURCE_DIR)/unpacked 
	cd $(SOURCE_DIR)
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make $(ROS_PARALLEL_JOBS) && touch src/osgOcean.pc && make install
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
