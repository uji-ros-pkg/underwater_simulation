/// Source code for the Profiling Sonar Sensor.
/**
 * Made by Olaya Alvarez
 * Modification from a file by
 * Mario Prats and Javier Perez.
 * This file contains the source code of the functions defined
 * in the header file.
 * Here are defined all the functions that make the sensor work.
 */

#include <pluginlib/class_list_macros.h>
#include <uwsim/ProfilingSonar.h>
#include <osg/PositionAttitudeTransform>
// For ROS interfaces
#include <sensor_msgs/LaserScan.h>
#include <typeinfo>

const long double PI = 3.141592653589793238L;

osg::Node *trackNode_;

void ProfilingSonar::ProfilingSonarSensing(osg::Group *uwsim_root, std::string name,
                                           std::string parentName, osg::Node *trackNode,
                                           double initAngle, double finalAngle, double alpha,
                                           double range, double fov, unsigned int mask,
                                           int visible,unsigned int ARMask)
{
  // Decide number of cameras to use and pixels for each camera
  camsFOV = fov;
  nCams=(int)(finalAngle-initAngle)/fov+1;
  camPixels=fov / alpha;

  // Set pose of each camera, add cameras to virtual camera from OSG node
  for(int i=0;i<nCams;i++)
  {
    // Set pose
    osg::PositionAttitudeTransform * mTc= new osg::PositionAttitudeTransform;
    mTc->setPosition(osg::Vec3d(0,0,0));
    mTc->setAttitude(osg::Quat( (initAngle+fov + fov*i)* PI /180.0 , osg::Vec3d(1,0,0)));
    trackNode->asTransform()->addChild(mTc);
    // Add to OSG virtual camera
    vcams.push_back(VirtualCamera(uwsim_root, name,parentName, mTc, camPixels, fov, range));
  }

  // Give to class all the new obtained values
  this->numpixels = fabs(finalAngle - initAngle) / alpha + 1;
  this->range = range;
  this->initAngle = initAngle;
  this->finalAngle = finalAngle;
  this->angleIncr = alpha;
  this->name=name;
  parentLinkName=parentName;

  // Call function which configures cameras
  preCalcTable();

  // Obtain mask for each camera. The mask indicates which layers the camera is detecting
  // If set to FFFF, detects water particles
  for(int i=0;i<nCams;i++)
  {
    vcams[i].textureCamera->setCullMask(mask);
  }

  // Draws rays on scene if required
  if (visible)
  {
    osg::ref_ptr<osg::Geometry> beam = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
    osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
    for(double initAux=initAngle;initAux<finalAngle;initAux+=angleIncr)
    {
      osg::Vec3d start(0, 0, 0);
      osg::Vec3d end(0, sin(initAux*3.14/180.0)*range, -cos(initAux*3.14/180.0)*range);
      points->push_back(start);
      points->push_back(end);
    }
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(0.0, 1.0, 0.0, 0.6));
    beam->setVertexArray(points.get());
    beam->setColorArray(color.get());
    beam->setColorBinding(osg::Geometry::BIND_OVERALL);
    beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, points->size()));
    geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    geode->addDrawable(beam.get());
    geode->setNodeMask(ARMask);
  }
  trackNode->asGroup()->addChild(geode);
  trackNode_ = trackNode;
}



void ProfilingSonar::preCalcTable()
{

  int iCam=0;
  remapVector.resize(numpixels);
  int current = 0;
  double lastTheta = 0;
  double thetacenter;
  osg::Vec3d first, last, center;
  osg::Matrix *MVPW;

  for (int i = 0; i < nCams*camPixels; i++)
  {
      // True only when we are configuring a new camera
      // That is, if camera has 2 pixels, enters here every 2 times
      if(i>=camPixels*iCam)
      {
          //Create matrix to unproject camera points to real world
          MVPW = new osg::Matrix(
                    vcams[iCam].textureCamera->getViewMatrix() * vcams[iCam].textureCamera->getProjectionMatrix()
                    * vcams[iCam].textureCamera->getViewport()->computeWindowMatrix());
          MVPW->invert(*MVPW);

          //Get first last and center points from camera
          first = osg::Vec3d(0, 0, 1) * (*MVPW) ;
          last = osg::Vec3d(0, camPixels - 1, 1) * (*MVPW);
          center = osg::Vec3d(0, camPixels / 2, 1) * (*MVPW);
          thetacenter = acos((first * center) / (center.length() * first.length())) + camsFOV*iCam*PI/180;

          iCam++;
       }

    //Interpolate points
    osg::Vec3d point = osg::Vec3d(0, i%camPixels , 1) * (*MVPW);

    double theta = acos((first * point) / (first.length() * point.length())) + camsFOV*(iCam-1)*PI/180;

    // Adding distortion to camera
    while (theta >= angleIncr * current*PI/180 && current < numpixels)
    {
      if (theta == angleIncr * current*PI/180 )
      { //usually only first iteration as point has to be exactly the same
        remapVector[current].pixel1 = i;
        remapVector[current].weight1 = 0.50;
        remapVector[current].pixel2 = i;
        remapVector[current].weight2 = 0.50;
      }
      else
      { //Interpolate between this and last point
        double dist = fabs(theta - angleIncr * current*PI/180 ), prevdist = fabs(lastTheta - angleIncr * current*PI/180 );
        remapVector[current].pixel1 = i;
        remapVector[current].weight1 = prevdist / (dist + prevdist);
        remapVector[current].pixel2 = i - 1;
        remapVector[current].weight2 = dist / (dist + prevdist);
      }
      remapVector[current].distort = 1 / cos(fabs(theta - thetacenter));

      current++;


    }
    lastTheta = theta;

  }

}

int ProfilingSonar::getTFTransform(tf::Pose & pose, std::string & parent)
{
    // parent from where we are computing the sensor position
    parent=relativeTo;
    // Pose of the sensor according to relative position from parent
    pose.setOrigin(tf::Vector3(trackNode_->asTransform()->asPositionAttitudeTransform()->getPosition().x(),
                   trackNode_->asTransform()->asPositionAttitudeTransform()->getPosition().y(),
                   trackNode_->asTransform()->asPositionAttitudeTransform()->getPosition().z()));
    pose.setRotation(tf::Quaternion(trackNode_->asTransform()->asPositionAttitudeTransform()->getAttitude().x(),
                     trackNode_->asTransform()->asPositionAttitudeTransform()->getAttitude().y(),
                     trackNode_->asTransform()->asPositionAttitudeTransform()->getAttitude().z(),
                     trackNode_->asTransform()->asPositionAttitudeTransform()->getAttitude().w()));

    // The TF convention is different to OSG one
    tf::Pose OSGToTFconvention;
    OSGToTFconvention.setOrigin(tf::Vector3(0,0,0));
    OSGToTFconvention.setRotation(tf::Quaternion(tf::Vector3(0,1,0),PI/2));  //As we are using camera to simulate it, we need to rotate it
    pose=pose*OSGToTFconvention;

    return 1;
}



//Create the barebone

ProfilingSonar::ProfilingSonar(ProfilingSonar_Config * cfg, osg::Node *trackNode) : SimulatedDevice(cfg)
{
    // Set position of the sensor according to declaration in
    // xml with relativeTo variable
    this->parent = trackNode;
    this->relativeTo = cfg->relativeTo;
    this->position[0] = cfg->position[0];
    this->position[1] = cfg->position[1];
    this->position[2] = cfg->position[2];
    this->orientation[0] = cfg->orientation[0];
    this->orientation[1] = cfg->orientation[1];
    this->orientation[2] = cfg->orientation[2];
}



SimulatedDeviceConfig::Ptr ProfilingSonar_Factory::processConfig(const xmlpp::Node* node,
                                                                 ConfigFile * config)
{
    //Declare an object of the class created to hold the configuration
    ProfilingSonar_Config * cfg = new ProfilingSonar_Config(getType());
    xmlpp::Node::NodeList list = node->get_children();
    //Looping the XML node received as argument
    //We iterate through the list of sons of the node which contains the properties assigned in the DTD
    for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter !=list.end(); ++iter)
    {
        const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    //We dont parse name because it is already parsed by the mother
        if (child->get_name() == "relativeTo")
            config->extractStringChar(child,cfg->relativeTo);
        if (child->get_name() == "linkName")
            config->extractStringChar(child,cfg->linkName);
        else if (child->get_name() == "visible")
            config->extractIntChar(child, cfg->visible);
        else if (child->get_name() == "position")
            config->extractPositionOrColor(child, cfg->position);
        else if (child->get_name() == "orientation")
            config->extractOrientation(child, cfg->orientation);
        else if (child->get_name() == "initAngle")
            config->extractFloatChar(child, cfg->initAngle);
        else if (child->get_name() == "finalAngle")
            config->extractFloatChar(child, cfg->finalAngle);
        else if (child->get_name() == "angleIncr")
            config->extractFloatChar(child, cfg->angleIncr);
        else if (child->get_name() == "range")
            config->extractFloatChar(child, cfg->range);
        else if (child->get_name() == "fov")
            config->extractFloatChar(child, cfg->fov);
        else if (child->get_name() == "angleIncr")
            config->extractFloatChar(child, cfg->angleIncr);
    }
    return SimulatedDeviceConfig::Ptr(cfg);
}


// Actually move from the config object to the device object
// This function contains the sensor configuration, and finally sets the position of the device
// Here we have the necessary methods to create the OSG node that does the sensing
bool ProfilingSonar_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars,
                                         SceneBuilder *sceneBuilder, size_t iteration)
{
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      ProfilingSonar_Config * cfg = dynamic_cast<ProfilingSonar_Config *>(vehicleChars.simulated_devices[i].get());
      osg::ref_ptr<osg::Node> target;
      if (cfg)
      {

    int target=-1;
    for(int j=0;j<auv->urdf->link.size();j++)
    {
        if(auv->urdf->link[j]->getName()==cfg->relativeTo)
        {
                target=j;
        }
    }
    if(target==-1)
    {
        OSG_FATAL << "ProfilingSonar device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
          << vehicleChars.name << "' has an unknown relativeTo, discarding..." << std::endl;
    }
    else
    {
        // Code that creates the OSG nodes
        OSG_INFO << "Adding a Profiling Sensor..."<< std::endl;

        // Code to set the position of the device
        osg::ref_ptr < osg::Transform > vMd = (osg::Transform*) new osg::PositionAttitudeTransform;
        vMd->asPositionAttitudeTransform()->setPosition(osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
        vMd->asPositionAttitudeTransform()->setAttitude(
             osg::Quat(cfg->orientation[0], osg::Vec3d(1, 0, 0), cfg->orientation[1], osg::Vec3d(0, 1, 0),
             cfg->orientation[2], osg::Vec3d(0, 0, 1)));
        auv->urdf->link[target]->getParent(0)->getParent(0)->asGroup()->addChild(vMd);

        unsigned int mask;
        mask = sceneBuilder->scene->getOceanScene()->getNormalSceneMask(); // by default

        // Create an object ProfilingSonar

       ProfilingSonar* PS;
       PS = new ProfilingSonar(cfg, vMd);

       // This function must be called from here because is in cfg where we have the sensor parameters
       PS->ProfilingSonarSensing(sceneBuilder->root, cfg->name, cfg->linkName, vMd,
                                                 cfg->initAngle, cfg->finalAngle, cfg->angleIncr,
                                                 cfg->range, cfg->fov, mask, cfg->visible, mask);

       // Upload configuration to the AUV devices
        auv->devices->all.push_back(ProfilingSonar::Ptr(PS));

        // Upload to camview, where all the cameras are
        for(unsigned int i=0;i<PS->nCams;i++)
         {
            auv->camview.push_back(PS->vcams[i]); // camview is a member of the class SimulatedIAUV
         }

    }// else target==-1
      }//if cfg
      else
        OSG_FATAL << "ProfilingSonar device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
            << vehicleChars.name << "' has empty cfg, discarding..." << std::endl;
    }
  return true;
}

/// ROS interface functions

std::vector<boost::shared_ptr<ROSInterface> > ProfilingSonar_Factory::getInterface(ROSInterfaceInfo & rosInterface,
                                                                                   std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
    ROS_INFO("getinterface");
    std::vector < boost::shared_ptr<ROSInterface> > ifaces;
    for (size_t i = 0; i < iauvFile.size(); ++i)
      for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
        if (iauvFile[i]->devices->all[d]->getType() == this->getType()
            && iauvFile[i]->devices->all[d]->name == rosInterface.targetName)
        {
          ifaces.push_back(
              boost::shared_ptr < ROSInterface
                  > (new ProfilingSonar_ROSPublisher(dynamic_cast<ProfilingSonar*>(iauvFile[i]->devices->all[d].get()),
                                                     rosInterface.topic, rosInterface.rate)));
        }
    if (ifaces.size() == 0)
      ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());



    return ifaces;
}


void ProfilingSonar_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("ProfilingSonar_ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::LaserScan > (topic, 1);
}

void ProfilingSonar_ROSPublisher::publish()
{
    sensor_msgs::LaserScan ls;

    // Pass all the sensor arguments to the ROS format
    if (dev != NULL)
    {

      ls.header.stamp = getROSTime();
      ls.header.frame_id = dev->name;

      double fov, aspect, near, far;

      ls.range_min = near;
      ls.range_max = dev->range; //far plane should be higher (z-buffer resolution)
      ls.angle_min = dev->initAngle * PI / 180;
      ls.angle_max = dev->finalAngle * PI / 180;
      ls.angle_increment = dev->angleIncr * PI / 180;

      std::vector<double> tmp;
      tmp.resize(dev->camPixels*dev->nCams);
     for(unsigned int j=0; j<dev->nCams ;j++)
      {
        dev->vcams[j].textureCamera->getProjectionMatrixAsPerspective(fov, aspect, near, far);

        float * data = (float *)dev->vcams[j].depthTexture->data();
        double a = far / (far - near);
        double b = (far * near) / (near - far);

        for (int i = 0; i < dev->camPixels; i++)
        {
          double Z = (data[i]); ///4294967296.0;
          tmp[i+dev->camPixels*j] = b / (Z - a);
        }
      }

      ls.ranges.resize(dev->numpixels);
      for (int i = 0; i < dev->numpixels; i++)
      {
        ls.ranges[i] = (tmp[dev->remapVector[i].pixel1] * dev->remapVector[i].weight1
            + tmp[dev->remapVector[i].pixel2] * dev->remapVector[i].weight2) * dev->remapVector[i].distort;
        if (ls.ranges[i] > dev->range)
          ls.ranges[i] = dev->range;
      }

      pub_.publish(ls);


      // Publish the frames to include Profiling Sonar in the TF tree
      tf::Pose pose;
      std::string parent;

      if(dev->getTFTransform(pose,parent))
      {
          tf::StampedTransform t(pose, ROSInterface::getROSTime(),
                                 "/girona500/" + dev->relativeTo,
                                 dev->name);

          boost::shared_ptr<tf::TransformBroadcaster> tfpub (new tf::TransformBroadcaster);

          tfpub->sendTransform(t);
      }


     }//if dev
    else
        std::cout << "dev == NULL";
}




#if ROS_VERSION_MINIMUM(1,9,0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(ProfilingSonar_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(ProfilingSonar_Factory, ProfilingSonar_Factory, uwsim::SimulatedDeviceFactory)
#endif
