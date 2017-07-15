#include <pluginlib/class_list_macros.h>
#include <uwsim/CommsDevice.h>
#include <osg/Node>
#include <uwsim/SimulatedIAUV.h>
#include <osg/PositionAttitudeTransform>
#include <thread>

/* You will need to add your code HERE */
#include <tf/transform_broadcaster.h>
#include <thread>
#include <uwsim/UWSimUtils.h>

SimulatedDeviceConfig::Ptr CommsDevice_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
  CommsDevice_Config * cfg = new CommsDevice_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();


  cfg->intrinsicDelay = 0;
  cfg->trTimeSd = 0;
  cfg->maxDistance = 99999999;
  cfg->minDistance = 0;
  cfg->pktErrRatioIncPerMeter = 0;
  cfg->minPktErrRatio = 0;
  cfg->devClass = 0;
  cfg->prTimeIncPerMeter = 0;

  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {

    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "relativeTo")
      config->extractStringChar(child, cfg->relativeTo);
    if (child->get_name() == "name")
      config->extractStringChar(child, cfg->name);
    else if (child->get_name() == "position")
      config->extractPositionOrColor(child, cfg->position);
    else if (child->get_name() == "orientation")
      config->extractOrientation(child, cfg->orientation);
    else if (child->get_name() == "class")
      config->extractUIntChar (child, cfg->devClass);
    else if (child->get_name() == "tfId")
      config->extractStringChar (child, cfg->tfId);
    else if (child->get_name() == "relativeTfId")
      config->extractStringChar (child, cfg->relativeTfId);
    else if (child->get_name() == "maxBitRate")
      config->extractUIntChar (child, cfg->maxBitRate);
    else if (child->get_name() == "intrinsicDelay")
      config->extractUIntChar (child, cfg->intrinsicDelay);
    else if (child->get_name() == "prTimeIncPerMeter")
      config->extractFloatChar (child, cfg->prTimeIncPerMeter);
    else if (child->get_name() == "trTime")
      config->extractFloatChar (child, cfg->trTime);
    else if (child->get_name() == "trTimeSd")
      config->extractFloatChar (child, cfg->trTimeSd);
    else if (child->get_name() == "maxDistance")
      config->extractUIntChar (child, cfg->maxDistance);
    else if (child->get_name() == "minDistance")
      config->extractUIntChar (child, cfg->minDistance);
    else if (child->get_name() == "mac")
      config->extractUIntChar (child, cfg->mac);
    else if (child->get_name() == "pktErrRatioIncPerMeter")
      config->extractFloatChar (child, cfg->prTimeIncPerMeter);
    else if (child->get_name() == "minPktErrRatio")
      config->extractFloatChar (child, cfg->minPktErrRatio);
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool CommsDevice_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration)
{

  if (iteration > 0)
    return true;

  ROS_INFO("CommsDevice: applyConfig method");
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      CommsDevice_Config * cfg = dynamic_cast<CommsDevice_Config *>(vehicleChars.simulated_devices[i].get());
      ROS_INFO("CommsDevice: Checking configuration");
      if (cfg)
      {
          int target=-1;
          for(int j=0;j<auv->urdf->link.size();j++)
          {
            if(auv->urdf->link[j]->getName()==cfg->relativeTo)
            {
              target=j;
              ROS_INFO("CommsDevice: target assignet");
            }
          }
          if(target==-1)
          {
            OSG_FATAL << "CommsDevice device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
                    << vehicleChars.name << "' has an unknown relativeTo, discarding..." << std::endl;
          }
          else
          {
            ROS_INFO("CommsDevice: creating urdf link");
            osg::ref_ptr < osg::Transform > vMd = (osg::Transform*) new osg::PositionAttitudeTransform;
            vMd->asPositionAttitudeTransform()->setPosition(osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
            vMd->asPositionAttitudeTransform()->setAttitude(
                       osg::Quat(cfg->orientation[0], osg::Vec3d(1, 0, 0), cfg->orientation[1], osg::Vec3d(0, 1, 0),
                       cfg->orientation[2], osg::Vec3d(0, 0, 1)));
            auv->urdf->link[target]->getParent(0)->getParent(0)->asGroup()->addChild(vMd);
            auto dev = new CommsDevice(cfg, auv->urdf->link[target], auv);
            auv->devices->all.push_back(CommsDevice::Ptr(dev));
            
            osg::ref_ptr<osg::Node> model = UWSimGeometry::createOSGCylinder(0.06,0.3);
            vMd->addChild(model.get());
            ROS_INFO("CommsDevice: added successfully");
            dev->Start();
          }
      }
      else
        OSG_FATAL << "CommsDevice device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
            << vehicleChars.name << "' has empty cfg, discarding..." << std::endl;
    }


  return true;
}

std::vector<boost::shared_ptr<ROSInterface> > CommsDevice_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector < boost::shared_ptr<ROSInterface> > ifaces;
  for (size_t i = 0; i < iauvFile.size(); ++i)
    for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
      if (iauvFile[i]->devices->all[d]->getType() == this->getType()
          && iauvFile[i]->devices->all[d]->name == rosInterface.targetName)
      {
        ifaces.push_back(
            boost::shared_ptr < ROSInterface
                > (new CommsDevice_ROSPublisher(dynamic_cast<CommsDevice*>(iauvFile[i]->devices->all[d].get()),
                                                rosInterface.topic, rosInterface.rate)));
      }
  if (ifaces.size() == 0)
    ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());
  return ifaces;
}

bool CommsDevice::_Check()
{
  bool res = true;
  dccomms_ros_msgs::CheckDevice srv;

  srv.request.iddev = this->config->name;

  if(!_checkService.call(srv))
  {
    res = false;
  }

  return res && srv.response.exists;
}

bool CommsDevice::_Remove()
{
  bool res = true;
  dccomms_ros_msgs::RemoveDevice srv;

  srv.request.iddev = this->config->name;

  if(!_rmService.call(srv))
  {
    res = false;
  }

  return res && srv.response.removed;
}

bool CommsDevice::_Add()
{
  dccomms_ros_msgs::AddDevice srv;

  srv.request.frameId = this->config->tfId;
  srv.request.iddev = this->config->name;
  srv.request.mac = this->config->mac;
  srv.request.maxBitRate = this->config->maxBitRate;
  srv.request.maxDistance = this->config->maxDistance;
  srv.request.minDistance = this->config->minDistance;
  srv.request.minPktErrorRate = this->config->minPktErrRatio;
  srv.request.pktErrorRateIncPerMeter = this->config->pktErrRatioIncPerMeter;
  srv.request.prTimeIncPerMeter = this->config->prTimeIncPerMeter;
  srv.request.trTimeMean = this->config->trTime;
  srv.request.trTimeSd = this->config->trTimeSd;
  srv.request.devType = this->config->devClass;


  ROS_INFO ("CommsDevice  ID = %s ; Frame = %s", srv.request.iddev.c_str (), srv.request.frameId.c_str());
  if(!_addService.call(srv))
  {
    ROS_ERROR("fail adding '%s'", srv.request.iddev.c_str ());
    return false;
  }
  else
  {
    ROS_INFO("CommsDevice '%s' added", srv.request.iddev.c_str ());
    return true;
  }
}

void CommsDevice::Start()
{
  auto netSimInterfaceWork = [this](void)
  {
    while(1)
    {
      if(!_Check())
      {
        _Remove();
        _Add();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds((int) 2000));
    }
  };
  std::thread starter(netSimInterfaceWork);
  starter.detach();
}

CommsDevice::CommsDevice(CommsDevice_Config * cfg, osg::ref_ptr<osg::Node> target, SimulatedIAUV * auv) :
    SimulatedDevice(cfg)
{
  if(cfg->tfId.length() == 0)
  {
    tfId = std::string(auv->name) + "/"+cfg->name;
  }
  else
  {
    tfId = cfg->tfId;
  }

  if(cfg->relativeTfId.length() != 0)
  {
    targetTfId = cfg->relativeTfId;
  }
  else
  {
    targetTfId = std::string(auv->name);
    if(cfg->relativeTo.length() != 0)
    {
      targetTfId += "/" + cfg->relativeTo;
    }
  }

  ROS_INFO("CommsDevice targetTfId: '%s' ; tfId: '%s'",
           targetTfId.c_str (),
           tfId.c_str());

  _addService = this->node.serviceClient<dccomms_ros_msgs::AddDevice>("/dccomms_netsim/add_net_device");
  _rmService = this->node.serviceClient<dccomms_ros_msgs::RemoveDevice>("/dccomms_netsim/remove_net_device");
  _checkService = this->node.serviceClient<dccomms_ros_msgs::CheckDevice>("/dccomms_netsim/check_net_device");

  this->config = cfg;
  this->parent = target;
  this->auv = auv;
}

void CommsDevice_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("CommsDevice ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise < geometry_msgs::Pose > (topic, 1);
}

void CommsDevice_ROSPublisher::publish()
{
  geometry_msgs::Pose msg;
  boost::shared_ptr<osg::Matrix> mat = getWorldCoords(dev->parent);
  auto trans = mat->getTrans();
  auto rotate = mat->getRotate();
  msg.position.x=trans.x();
  msg.position.y=trans.y();
  msg.position.z=trans.z();
  msg.orientation.x=rotate.x();
  msg.orientation.y=rotate.y();
  msg.orientation.z=rotate.z();
  msg.orientation.w=rotate.w();
  pub_.publish(msg);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(this->dev->config->position[0],
                                  this->dev->config->position[1],
                                  this->dev->config->position[2]));
  transform.setRotation(tf::Quaternion(
                          this->dev->config->orientation[0],
                          this->dev->config->orientation[1],
                          this->dev->config->orientation[2]));
  //ROS_INFO("CommsDevice Publisher tfId: %s ; auv: %s", dev->tfId.c_str (), dev->auv->name.c_str ());
  _tfBr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), dev->targetTfId, dev->tfId));


}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(CommsDevice_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(CommsDevice_Factory, CommsDevice_Factory, uwsim::SimulatedDeviceFactory)
#endif
