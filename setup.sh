#!/bin/bash

apt-get install libxml++2.6-dev libmuparser-dev libopenscenegraph-dev libfftw3-dev geographiclib-tools libgeographic-dev geographiclib-doc -y

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

chmod u+x install_geographiclib_datasets.sh

./install_geographiclib_datasets.sh


