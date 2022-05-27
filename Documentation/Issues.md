
# Issues

## Exception `GeographicLib::GeographicErr`

*when starting `uwsim`.*

### Showcase

*You should be getting the following error* <br>
*when executing `rosrun uwsim uwsim` :*

```txt
...
data/objects/gun.osg
Starting UWSim...
Loading SimulatedDevices plugin: 'AcousticCommsDevice_Factory'
Loading SimulatedDevices plugin: 'CustomCommsDevice_Factory'
Loading SimulatedDevices plugin: 'DredgeTool_Factory'
Loading SimulatedDevices plugin: 'ForceSensor_Factory'
Loading SimulatedDevices plugin: 'SimDev_Echo_Factory'
. Setting localized world: 7.1e-05s
terminate called after throwing an instance of 'GeographicLib::GeographicErr'
what():  File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm
/opt/ros/melodic/lib/uwsim/uwsim: line 23:  1452 Aborted                 
(core dumped) rosrun uwsim uwsim_binary --dataPath ~/.uwsim/data $@
```

### Fix

*Run the following command:*

```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod u+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```