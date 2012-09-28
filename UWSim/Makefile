download_and_build: downloaded all

DATAFILE=UWSim-data.tgz

downloaded:
	wget http://uji-ros-pkg.googlecode.com/files/$(DATAFILE) && tar xvf $(DATAFILE) && rm $(DATAFILE)
	touch downloaded

include $(shell rospack find mk)/cmake.mk

