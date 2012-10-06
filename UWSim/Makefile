download_and_build: downloaded all

DATAFILE=UWSim-data.tgz
RVIZ_DATAFILE=UWSim-dta-rviz.tgz

downloaded:
	wget http://uji-ros-pkg.googlecode.com/files/$(DATAFILE) && tar xvf $(DATAFILE) && rm $(DATAFILE)
	touch downloaded

rviz-data:
	wget http://uji-ros-pkg.googlecode.com/files/$(RVIZ_DATAFILE) && tar xvf $(RVIZ_DATAFILE) && rm $(RVIZ_DATAFILE)

include $(shell rospack find mk)/cmake.mk

