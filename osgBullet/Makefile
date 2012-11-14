all: installed
  
FILENAME = osgBullet_02_00_00.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL= http://osgbullet.googlecode.com/files/$(FILENAME)
SOURCE_DIR = build/osgBullet_02_00_00
TARBALL_PATCH= osgBullet_02_00_00.patch
UNPACK_CMD = tar xvf
MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = `rospack find osgBullet`/
OSGWORKS_DIR = `rospack find osgWorks`/
BULLET_DIR = `rospack find bullet`/
CMAKE = cmake
CMAKE_ARGS= -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) \
-DBULLET_COLLISION_LIBRARY=$(BULLET_DIR)/lib/libBulletCollision.so \
-DBULLET_DYNAMICS_LIBRARY=$(BULLET_DIR)/lib/libBulletDynamics.so \
-DBULLET_MATH_LIBRARY=$(BULLET_DIR)/lib/libLinearMath.so \
-DBULLET_SOFTBODY_LIBRARY=$(BULLET_DIR)/lib/libBulletSoftBody.so \
-DBULLET_INCLUDE_DIRS=$(BULLET_DIR)/include \
-DBULLET_INCLUDE_DIR=$(BULLET_DIR)/include \
-DOSGWCONTROLS_LIBRARY=$(OSGWORKS_DIR)/lib/libosgwControls.so \
-DOSGWQUERY_LIBRARY=$(OSGWORKS_DIR)/lib/libosgwQuery.so \
-DOSGWTOOLS_LIBRARY=$(OSGWORKS_DIR)/lib/libosgwTools.so \
-DOSGWORKS_INCLUDE_DIR=$(OSGWORKS_DIR)/include \
-DOSGBULLET_BUILD_APPLICATIONS=OFF \
-DOSGBULLET_BUILD_EXAMPLES=OFF \
-DOSGBULLET_USE_DOUBLE_PRECISION=ON


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
