#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <uwsim/CommsDevice.h>
#include <uwsim/SimulatedIAUV.h>

/* You will need to add your code HERE */

#include <dccomms_packets/VariableLength2BPacket.h>
#include <dccomms_packets/VariableLengthPacket.h>
#include <thread>
#include <uwsim/UWSimUtils.h>

void CommsDevice_Factory::processCommonConfig(const xmlpp::Node *node,
                                              ConfigFile *config,
                                              CommsDevice_Config *cfg) {
  xmlpp::Node::NodeList list = node->get_children();


  cfg->macProtocol = "";
  cfg->macDistance = 0;

  cfg->txFifoSize = 500000;
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end();
       ++iter) {

    const xmlpp::Node *child = dynamic_cast<const xmlpp::Node *>(*iter);
    if (child->get_name() == "relativeTo")
      config->extractStringChar(child, cfg->relativeTo);
    if (child->get_name() == "dccommsId")
      config->extractStringChar(child, cfg->dccommsId);
    else if (child->get_name() == "position")
      config->extractPositionOrColor(child, cfg->position);
    else if (child->get_name() == "orientation")
      config->extractOrientation(child, cfg->orientation);
    else if (child->get_name() == "mesh")
      config->extractMesh(child, cfg->mesh);
    else if (child->get_name() == "tfId")
      config->extractStringChar(child, cfg->tfId);
    else if (child->get_name() == "relativeTfId")
      config->extractStringChar(child, cfg->relativeTfId);
    else if (child->get_name() == "mac")
      config->extractUIntChar(child, cfg->mac);
    else if (child->get_name() == "channelId")
      config->extractUIntChar(child, cfg->channelId);
    else if (child->get_name() == "txFifoSize")
      config->extractUIntChar(child, cfg->txFifoSize);
    else if (child->get_name() == "txPacketBuilder")
      config->processPacketBuilderConfig(child, cfg->txPacketBuilderConfig);
    else if (child->get_name() == "rxPacketBuilder")
      config->processPacketBuilderConfig(child, cfg->rxPacketBuilderConfig);
    else if (child->get_name() == "logLevel")
      config->extractStringChar(child, cfg->logLevel);
    else if (child->get_name() == "disable")
      config->extractIntChar(child, cfg->disable);
    else if (child->get_name() == "macProtocol") {
      xmlpp::Node::NodeList emAttributes = child->get_children();
      for (xmlpp::Node::NodeList::iterator subiter = emAttributes.begin();
           subiter != emAttributes.end(); ++subiter) {
        const xmlpp::Node *ema = dynamic_cast<const xmlpp::Node *>(*subiter);
        if (ema->get_name() == "name")
          config->extractStringChar(ema, cfg->macProtocol);
        else if (ema->get_name() == "maxDistance")
          config->extractDecimalChar(ema, cfg->macDistance);
      }
    }
  }
}

bool CommsDevice_Factory::applyConfig(SimulatedIAUV *auv, Vehicle &vehicleChars,
                                      SceneBuilder *sceneBuilder,
                                      size_t iteration) {

  if (iteration > 0)
    return true;

  ROS_INFO("CommsDevice: applyConfig method");
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType()) {
      CommsDevice_Config *cfg = dynamic_cast<CommsDevice_Config *>(
          vehicleChars.simulated_devices[i].get());
      ROS_INFO("CommsDevice: Checking configuration");
      if (cfg) {
        int target = -1;
        for (int j = 0; j < auv->urdf->link.size(); j++) {
          if (auv->urdf->link[j]->getName() == cfg->relativeTo) {
            target = j;
            ROS_INFO("CommsDevice: target assignet");
          }
        }
        if (target == -1) {
          OSG_FATAL << "CommsDevice device '"
                    << vehicleChars.simulated_devices[i]->name
                    << "' inside robot '" << vehicleChars.name
                    << "' has an unknown relativeTo, discarding..."
                    << std::endl;
        } else {
          ROS_INFO("CommsDevice: creating urdf link");
          osg::ref_ptr<osg::Transform> vMd =
              (osg::Transform *)new osg::PositionAttitudeTransform;
          vMd->asPositionAttitudeTransform()->setPosition(
              osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
          vMd->asPositionAttitudeTransform()->setAttitude(osg::Quat(
              cfg->orientation[0], osg::Vec3d(1, 0, 0), cfg->orientation[1],
              osg::Vec3d(0, 1, 0), cfg->orientation[2], osg::Vec3d(0, 0, 1)));

          auv->urdf->link[target]
              ->getParent(0)
              ->getParent(0)
              ->asGroup()
              ->addChild(vMd);

          auto dev = Create(cfg, auv->urdf->link[target], auv);
          auv->devices->all.push_back(UWSimCommsDevice::Ptr(dev));

          if (dev->render) {
            try {
              osg::ref_ptr<osg::Node> model =
                  osgDB::readNodeFile(dev->GetConfig()->mesh.path);

              if (model != NULL) {
                vMd->asPositionAttitudeTransform()->setScale(osg::Vec3d(
                    cfg->mesh.scaleFactor[0], cfg->mesh.scaleFactor[1],
                    cfg->mesh.scaleFactor[2]));
                model.get()->getOrCreateStateSet()->setMode(
                    GL_NORMALIZE, osg::StateAttribute::ON);

                vMd->addChild(model.get());
              } else {
                ROS_ERROR("CommsDevice ('%s'): mesh loading failed",
                          dev->name.c_str());
              }
            } catch (exception e) {
              ROS_ERROR("CommsDevice ('%s'): mesh loading failed: %s",
                        dev->name.c_str(), e.what());
            }
          }
          if (cfg->disable) {
            ROS_WARN("CommsDevice: disabled by user");
            continue;
          }
          ROS_INFO("CommsDevice: added successfully");
          dev->AddToNetSim();
        }
      } else
        OSG_FATAL << "CommsDevice device '"
                  << vehicleChars.simulated_devices[i]->name
                  << "' inside robot '" << vehicleChars.name
                  << "' has empty cfg, discarding..." << std::endl;
    }

  return true;
}

std::vector<std::shared_ptr<ROSInterface>> CommsDevice_Factory::getInterface(
    ROSInterfaceInfo &rosInterface,
    std::vector<std::shared_ptr<SimulatedIAUV>> &iauvFile) {
  std::vector<std::shared_ptr<ROSInterface>> ifaces;
  for (size_t i = 0; i < iauvFile.size(); ++i)
    for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
      if (iauvFile[i]->devices->all[d]->getType() == this->getType() &&
          iauvFile[i]->devices->all[d]->name == rosInterface.targetName) {
        ifaces.push_back(
            std::shared_ptr<ROSInterface>(new CommsDevice_ROSPublisher(
                dynamic_cast<UWSimCommsDevice *>(
                    iauvFile[i]->devices->all[d].get()),
                rosInterface.topic, rosInterface.rate)));
      }
  if (ifaces.size() == 0)
    ROS_WARN("Returning empty ROS interface for device %s...",
             rosInterface.targetName.c_str());
  return ifaces;
}

void UWSimCommsDevice::AddToNetSim() { _AddToNetSim(); }

void UWSimCommsDevice::SetPacketBuilder(PACKET_TYPE pType,
                                        CommsDevice_Config *cfg,
                                        PacketBuilderConfig *pbcfg) {
  auto netsim = uwsim::NetSim::GetSim();
  dccomms::PacketBuilderPtr pb;
  if (pbcfg->libPath == "") { // Default lib
    if (pbcfg->className == "DataLinkFrameBuilderCRC16")
      pb = std::shared_ptr<dccomms::DataLinkFrameBuilderCRC16>(
          new dccomms::DataLinkFrameBuilderCRC16());
    else if (pbcfg->className == "VariableLengthPacketBuilder")
      pb = std::shared_ptr<dccomms_packets::VariableLengthPacketBuilder>(
          new dccomms_packets::VariableLengthPacketBuilder());
    else if (pbcfg->className == "VariableLength2BPacketBuilder")
      pb = std::shared_ptr<dccomms_packets::VariableLength2BPacketBuilder>(
          new dccomms_packets::VariableLength2BPacketBuilder());
    else {
      ROS_ERROR("CommsDevice ('%s'): '%s' packet builder not found in "
                "default library ",
                cfg->dccommsId.c_str(), pbcfg->className.c_str());
      throw dccomms::CommsException(pbcfg->className,
                                    COMMS_EXCEPTION_CONFIG_ERROR);
    }
    if (pb) {
      netsim->SetPacketBuilder(cfg->dccommsId, pType, pb);
    }
  } else {
    std::string path = pbcfg->libPath;
    if (path[0] != '/') {
      path = std::string(getenv("HOME")) + "/.uwsim/data/netsim/" + path;
    }
    if (access(path.c_str(), F_OK) != -1) {
      // file exists
      try {
        netsim->SetPacketBuilder(cfg->dccommsId, pType, path, pbcfg->className);
      } catch (std::exception e) {
        ROS_ERROR("CommsDevice ('%s'): '%s' class implementation not loaded "
                  "(inner exception: '%s')",
                  cfg->dccommsId.c_str(), pbcfg->className.c_str(), e.what());
        throw dccomms::CommsException(pbcfg->className,
                                      COMMS_EXCEPTION_CONFIG_ERROR);
      }
    } else {
      // file doesn't exist
      ROS_ERROR("CommsDevice ('%s'): '%s' library does not exist",
                cfg->dccommsId.c_str(), path.c_str());
      throw dccomms::CommsException(path, COMMS_EXCEPTION_CONFIG_ERROR);
    }
  }
}

UWSimCommsDevice::UWSimCommsDevice(CommsDevice_Config *cfg,
                                   osg::ref_ptr<osg::Node> target,
                                   SimulatedIAUV *auv)
    : SimulatedDevice(cfg) {
  SetPacketBuilder(PACKET_TYPE::TX_PACKET, cfg, &cfg->txPacketBuilderConfig);
  SetPacketBuilder(PACKET_TYPE::RX_PACKET, cfg, &cfg->rxPacketBuilderConfig);
}

void UWSimCommsDevice::Init(CommsDevice_Config *cfg,
                            osg::ref_ptr<osg::Node> target,
                            SimulatedIAUV *auv) {
  if (cfg->tfId.length() == 0) {
    tfId = std::string(auv->name) + "/" + cfg->name;
  } else {
    tfId = cfg->tfId;
  }

  if (cfg->relativeTfId.length() != 0) {
    targetTfId = cfg->relativeTfId;
  } else {
    targetTfId = std::string(auv->name);
    if (cfg->relativeTo.length() != 0) {
      targetTfId += "/" + cfg->relativeTo;
    }
  }

  ROS_INFO("CommsDevice targetTfId: '%s' ; tfId: '%s'", targetTfId.c_str(),
           tfId.c_str());

  render = cfg->mesh.path.length() != 0;

  this->parent = target;
  this->auv = auv;
  this->name = cfg->dccommsId;
  SetConfig(cfg);
}
void CommsDevice_ROSPublisher::createPublisher(ros::NodeHandle &nh) {
  ROS_INFO("CommsDevice ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise<geometry_msgs::Pose>(topic, 1);
}

void CommsDevice_ROSPublisher::publish() {
  geometry_msgs::Pose msg;
  std::shared_ptr<osg::Matrix> mat = getWorldCoords(dev->parent);
  auto trans = mat->getTrans();
  auto rotate = mat->getRotate();
  msg.position.x = trans.x();
  msg.position.y = trans.y();
  msg.position.z = trans.z();
  msg.orientation.x = rotate.x();
  msg.orientation.y = rotate.y();
  msg.orientation.z = rotate.z();
  msg.orientation.w = rotate.w();
  pub_.publish(msg);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(this->dev->GetConfig()->position[0],
                                  this->dev->GetConfig()->position[1],
                                  this->dev->GetConfig()->position[2]));
  transform.setRotation(tf::Quaternion(this->dev->GetConfig()->orientation[0],
                                       this->dev->GetConfig()->orientation[1],
                                       this->dev->GetConfig()->orientation[2]));
  // ROS_INFO("CommsDevice Publisher tfId: %s ; auv: %s", dev->tfId.c_str (),
  // dev->auv->name.c_str ());
  _tfBr.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                           dev->targetTfId, dev->tfId));
}

//#if ROS_VERSION_MINIMUM(1, 9, 0)
//// new pluginlib API in Groovy and Hydro
// PLUGINLIB_EXPORT_CLASS(CommsDevice_Factory, uwsim::SimulatedDeviceFactory)
//#else
// PLUGINLIB_REGISTER_CLASS(CommsDevice_Factory, CommsDevice_Factory,
//                         uwsim::SimulatedDeviceFactory)
//#endif
