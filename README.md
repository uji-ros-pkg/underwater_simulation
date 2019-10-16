
# UWSim-NET: UWSim with Network Simulator
This is a fork of https://github.com/uji-ros-pkg/underwater_simulation.
This new version of UWSim intregates a Network Simulator to be used along with the dccomms API. This simulator uses the NS3 libraries and the AquaSim NG as a NS3 module. The documentation is a work-in-progress.

### Installation
1. Install system dependencies:
```bash
sudo apt-get install libxml++2.6-dev libmuparser-dev libopenscenegraph-dev libfftw3-dev geographiclib-tools libgeographic-dev geographiclib-doc -y
```
2. Install geographiclib datasets:
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod u+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```
3. Create a catkin workspace to build uwsim and place a .rosinstall file inside with the following contents:
```
- other: {local-name: /opt/ros/melodic/share/ros}
- other: {local-name: /opt/ros/melodic/share}
- other: {local-name: /opt/ros/melodic/stacks}
- setup-file: {local-name: /opt/ros/melodic/setup.sh}
- git: {local-name: src/uwsim_osgocean, uri: 'https://github.com/uji-ros-pkg/uwsim_osgocean.git', version: kinetic-devel}
- git: {local-name: src/uwsim_osgworks, uri: 'https://github.com/uji-ros-pkg/uwsim_osgworks.git', version: kinetic-devel}
- git: {local-name: src/uwsim_bullet, uri: 'https://github.com/uji-ros-pkg/uwsim_bullet.git', version: kinetic-devel}
- git: {local-name: src/uwsim_osgbullet, uri: 'https://github.com/uji-ros-pkg/uwsim_osgbullet.git', version: kinetic-devel}
- git: {local-name: src/visualization_osg, uri: 'https://github.com/uji-ros-pkg/visualization_osg.git', version: kinetic-devel}
- git: {local-name: src/underwater_simulation, uri: 'https://github.com/uji-ros-pkg/underwater_simulation.git', version: melodic-devel}
```    
4. Then, run the following commands to download the sources:
```bash
sudo apt-get install python-rosinstall
cd <UWSimWorkspace>
rosws update
```

5. Go again to src directory and clone the *dccomms_ros_pkgs* with the --recursive option (we recommand to first cache the github credentials):
```bash
cd <UWSimWorkspace>/src
git clone --recursive https://github.com/dcentelles/dccomms_ros_pkgs.git
```
4. Then go to the root of your catkin workspace and install the remaining ROS dependencies:
```bash
cd <UWSimWorkspace>
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r
```
5. Build and install the entire workspace with catkin_make_isolated:
```bash
catkin_make_isolated --install -j2
```
6. Source the catkin_workspace:
```bash
source install_isolated/setup.bash
```
9. Run the UWSim for the first time (if you have not already done so) and type Y to create the simulator directories:
```
rosrun uwsim uwsim
The UWSim data directory (~/.uwsim/data) does not exist. We need to download ~300Mb of data and place it under ~/.uwsim/data. Note this is required to run UWSim.
Continue (Y/n) ?
```
10. Close UWSim
