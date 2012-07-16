all: installed


SVN_URL=http://osgocean.googlecode.com/svn/trunk  
SVN_REVISION=-r256
SVN_DIR=build/osgOcean
SVN_PATCH= osgOcean-1.0.1.patch


include $(shell rospack find mk)/svn_checkout.mk

SOURCE_DIR=build/osgOcean
INSTALL_DIR = `rospack find osgOcean`/
CMAKE = cmake
CMAKE_ARGS= -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DUSE_FFTSS=OFF -DUSE_FFTW3F=ON /

installed: download
	cd $(SOURCE_DIR)
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make $(ROS_PARALLEL_JOBS) && touch src/osgOcean.pc && make install
	touch installed


clean:
	rm -rf lib
	rm -rf include
	rm -rf share
	rm -rf build
	rm -rf installed

