//"Echo" example, SimulatedDevice_Echo.cpp

#include <pluginlib/class_list_macros.h>
#include <uwsim/DredgeTool.h>

DredgeTool::DredgeTool(DredgeTool_Config * cfg, osg::ref_ptr<osg::Node> target) :
    SimulatedDevice(cfg)
{
  this->target = target;
}

//To be implemented
boost::shared_ptr<osg::Matrix> DredgeTool::getDredgePosition(){

  return getWorldCoords(target);;
}

void DredgeTool::dredgedParticles(int nparticles){

}




SimulatedDeviceConfig::Ptr DredgeTool_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
  DredgeTool_Config * cfg = new DredgeTool_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if(child->get_name() == "target")
      config->extractStringChar(child, cfg->target);
    else if(child->get_name() == "offsetp")
      config->extractPositionOrColor(child, cfg->offsetp);
    else if(child->get_name() == "offsetr")
      config->extractPositionOrColor(child, cfg->offsetr);
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool DredgeTool_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder,
                                      size_t iteration)
{
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      DredgeTool_Config * cfg = dynamic_cast<DredgeTool_Config *>(vehicleChars.simulated_devices[i].get());

      osg::ref_ptr<osg::Node> target;
      for(int j=0;j<auv->urdf->link.size();j++)
      {
        if(auv->urdf->link[j]->getName()==cfg->target)
        {
          target=auv->urdf->link[j];
        }
      }
      if(target)
        auv->devices->all.push_back(DredgeTool::Ptr(new DredgeTool(cfg,target)));
      else
        OSG_FATAL << "DredgeTool device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
            << vehicleChars.name << "' has empty info, discarding..." << std::endl;
    }
  return true;
}



#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(DredgeTool_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(DredgeTool_Factory, DredgeTool_Factory, uwsim::SimulatedDeviceFactory)
#endif

