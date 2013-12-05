/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */

#include <uwsim/ROSInterface.h>
#include <uwsim/UWSimUtils.h>
#include <osg/LineWidth>
#include <osg/Material>
#include <osgOcean/ShaderManager>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <underwater_sensor_msgs/Pressure.h>
#include <underwater_sensor_msgs/DVL.h>
#include <std_msgs/Bool.h>
#include <osg/LineStipple>
#include <robot_state_publisher/robot_state_publisher.h>

// static member
ros::Time ROSInterface::current_time_;

ROSSubscriberInterface::ROSSubscriberInterface(std::string topic) :
    ROSInterface(topic)
{
  startThread();
}

/* Thread code */
void ROSSubscriberInterface::run()
{
  ros::Duration(2).sleep();
  createSubscriber (nh_);
}

ROSSubscriberInterface::~ROSSubscriberInterface()
{
  join();
}

ROSOdomToPAT::ROSOdomToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName, double col[3],
                           int visualization, double max_waypoint_distance) :
    ROSSubscriberInterface(topic)
{
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
  started = 0; //Used in time
  trajectory_initialized = false;
  this->max_waypoint_distance = max_waypoint_distance;
  enable_visualization = (visualization >= 1);

  if (enable_visualization)
  {
    trajectory_points = new osg::Vec3Array;
    trajectory_points->push_back(osg::Vec3(0, 0, 0));
    trajectory = osg::ref_ptr < osg::Geometry > (new osg::Geometry());
    trajectory->setVertexArray(trajectory_points);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4f(col[0], col[1], col[2], 1.0f));
    trajectory->setColorArray(colors);
    trajectory->setColorBinding(osg::Geometry::BIND_OVERALL);
    prset = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);
    trajectory->addPrimitiveSet(prset);
    trajectory->setUseDisplayList(false);

    geode = osg::ref_ptr < osg::Geode > (new osg::Geode());
    geode->addDrawable(trajectory);
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(4.0f);

    //stipple for dashed lines:
    if (visualization > 1)
    {
      osg::LineStipple* linestipple = new osg::LineStipple;
      linestipple->setFactor(1);
      if (visualization == 2)
        linestipple->setPattern(0xf0f0);
      if (visualization == 3)
        linestipple->setPattern(0xff00);
      if (visualization == 4)
        linestipple->setPattern(0xf000);
      geode->getOrCreateStateSet()->setAttributeAndModes(linestipple, osg::StateAttribute::ON);
    }

    //Attach the trajectory to the localizedWorld node
    findNodeVisitor finder("localizedWorld");
    rootNode->accept(finder);
    std::vector<osg::Node*> node_list = finder.getNodeList();

    geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
    const std::string SIMULATOR_DATA_PATH = std::string(getenv("HOME")) + "/.uwsim/data";
    osgDB::Registry::instance()->getDataFilePathList().push_back(
        std::string(UWSIM_ROOT_PATH) + std::string("/data/shaders"));
    static const char model_vertex[] = "default_scene.vert";
    static const char model_fragment[] = "default_scene.frag";

    osg::ref_ptr < osg::Program > program = osgOcean::ShaderManager::instance().createProgram("robot_shader",
                                                                                              model_vertex,
                                                                                              model_fragment, "", "");
    program->addBindAttribLocation("aTangent", 6);

    geode->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
    geode->getStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
    geode->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
    node_list[0]->asGroup()->addChild(geode);
  }
}

void ROSOdomToPAT::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSOdomToPAT subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < nav_msgs::Odometry > (topic, 10, &ROSOdomToPAT::processData, this);
  if (sub_ == ros::Subscriber())
  {
    ROS_ERROR("ROSOdomToPAT::createSubscriber cannot subscribe to topic %s", topic.c_str());
  }
}

void ROSOdomToPAT::processData(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (transform != NULL)
  {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;
    //If velocity is zero, use the position reference. If not, use velocity
    //If position is not zero, use the position reference. If not, use velocity
    if (odom->pose.pose.orientation.x != 0 || odom->pose.pose.orientation.y != 0 || odom->pose.pose.orientation.z != 0
        || odom->pose.pose.position.x != 0 || odom->pose.pose.position.y != 0 || odom->pose.pose.position.z != 0)
    {
      sMsv_osg.setTrans(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
      sMsv_osg.setRotate(
          osg::Quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                    odom->pose.pose.orientation.w));

      //Store trajectory
      if (enable_visualization)
      {
        if (trajectory_initialized)
        {
          if ((trajectory_points->back() - sMsv_osg.getTrans()).length() > max_waypoint_distance)
          {
            trajectory_points->push_back(
                osg::Vec3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
            trajectory->setVertexArray(trajectory_points);
            ((osg::DrawArrays*)prset)->setFirst(0);
            ((osg::DrawArrays*)prset)->setCount(trajectory_points->size());
            std::cerr << "Trajectory_points size: " << trajectory_points->size() << std::endl;
          }
        }
        else
        {
          trajectory_points->clear();
          trajectory_points->push_back(
              osg::Vec3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
          trajectory_initialized = true;
        }
      }
    }
    else
    {
      //Get the current vehicle position and attitude in an homogeneous matrix
      sMsv_osg = transform->getMatrix();

      //Time issues
      double elapsed = 0;
      if (started != 0)
      {
        ros::WallDuration t_diff = ros::WallTime::now() - last;
        elapsed = t_diff.toSec();
        //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
        if (elapsed > 1)
          elapsed = 0;
      }
      started = 1;
      last = ros::WallTime::now();
      //std::cout<<elapsed<<std::endl;

      osg::Matrixd vMnv;
      osg::Matrixd T, Rx, Ry, Rz;
      T.makeTranslate(odom->twist.twist.linear.x * elapsed, odom->twist.twist.linear.y * elapsed,
                      odom->twist.twist.linear.z * elapsed);
      Rx.makeRotate(odom->twist.twist.angular.x * elapsed, 1, 0, 0);
      Ry.makeRotate(odom->twist.twist.angular.y * elapsed, 0, 1, 0);
      Rz.makeRotate(odom->twist.twist.angular.z * elapsed, 0, 0, 1);
      vMnv = Rz * Ry * Rx * T;

      sMsv_osg = vMnv * sMsv_osg;
    }
    transform->setMatrix(sMsv_osg);
  }
}

void ROSOdomToPAT::clearWaypoints()
{
  trajectory_points->clear();
  ((osg::DrawArrays*)prset)->setFirst(0);
  ((osg::DrawArrays*)prset)->setCount(trajectory_points->size());
  trajectory_initialized = false;
}

ROSOdomToPAT::~ROSOdomToPAT()
{
}

ROSTwistToPAT::ROSTwistToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName) :
    ROSSubscriberInterface(topic)
{
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
  started = 0; //used in time
}

void ROSTwistToPAT::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSTwistToPAT subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < geometry_msgs::TwistStamped > (topic, 10, &ROSTwistToPAT::processData, this);
}

void ROSTwistToPAT::processData(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
  if (transform != NULL)
  {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;
    //Get the current vehicle position and attitude in an homogeneous matrix
    sMsv_osg = transform->getMatrix();

    //Time issues
    double elapsed = 0;
    if (started != 0)
    {
      ros::WallDuration t_diff = ros::WallTime::now() - last;
      elapsed = t_diff.toSec();
      //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
      if (elapsed > 1)
        elapsed = 0;
    }
    started = 1;
    last = ros::WallTime::now();

    osg::Matrixd vMnv;
    osg::Matrixd T, Rx, Ry, Rz;
    T.makeTranslate(twist->twist.linear.x * elapsed, twist->twist.linear.y * elapsed, twist->twist.linear.z * elapsed);
    Rx.makeRotate(twist->twist.angular.x * elapsed, 1, 0, 0);
    Ry.makeRotate(twist->twist.angular.y * elapsed, 0, 1, 0);
    Rz.makeRotate(twist->twist.angular.z * elapsed, 0, 0, 1);
    vMnv = Rz * Ry * Rx * T;

    sMsv_osg = vMnv * sMsv_osg;

    transform->setMatrix(sMsv_osg);
  }
}

ROSTwistToPAT::~ROSTwistToPAT()
{
}

ROSPoseToPAT::ROSPoseToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName) :
    ROSSubscriberInterface(topic)
{
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
}

void ROSPoseToPAT::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSPoseToPAT subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < geometry_msgs::Pose > (topic, 10, &ROSPoseToPAT::processData, this);
}

void ROSPoseToPAT::processData(const geometry_msgs::Pose::ConstPtr& pose)
{
  if (transform != NULL)
  {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;

    sMsv_osg.setTrans(pose->position.x, pose->position.y, pose->position.z);
    sMsv_osg.setRotate(osg::Quat(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w));

    transform->setMatrix(sMsv_osg);

  }
}

ROSPoseToPAT::~ROSPoseToPAT()
{
}

/*
 class ROSNavigationDataToPAT: public ROSSubscriberInterface {
 osg::PositionAttitudeTransform *transform;
 public:
 ROSNavigationDataToPAT(std::string topic, osg::PositionAttitudeTransform *t): ROSSubscriberInterface(topic) {
 transform=t;
 }

 virtual void createSubscriber(ros::NodeHandle &nh) {
 sub_ = nh.subscribe<cola2_common::NavigationData>(topic, 10, &ROSNavigationDataToPAT::processData, this);
 }

 virtual void processData(const cola2_common::NavigationData::ConstPtr& odom) {
 //Simulated vehicle frame wrt real vehicle frame
 vpHomogeneousMatrix vMsv(0.8,0,0.8,0,M_PI,0);

 vpHomogeneousMatrix sMsv;


 //Set a position reference
 //Pose of the real vehicle wrt to the localization origin
 vpRotationMatrix pRv(vpRxyzVector(odom->pose[3],odom->pose[4],odom->pose[5]));
 vpTranslationVector pTv(odom->pose[0],odom->pose[1],odom->pose[2]);
 vpHomogeneousMatrix pMv;
 pMv.buildFrom(pTv, pRv);

 //Localization origin wrt simulator origin
 vpRxyzVector sRVp(M_PI,0,-M_PI_2);
 vpRotationMatrix sRp(sRVp);
 vpTranslationVector sTp(-514921,-4677958,3.4);

 vpHomogeneousMatrix sMp;
 sMp.buildFrom(sTp,sRp);

 sMsv=sMp*pMv*vMsv;

 if (transform!=NULL) {
 //OSG_DEBUG << "SimulatedVehicle::processData baseTransform not null" << std::endl;
 transform->setPosition(osg::Vec3d(sMsv[0][3],sMsv[1][3],sMsv[2][3]));
 vpRotationMatrix mr;
 sMsv.extract(mr);
 vpRxyzVector vr(mr);
 osg::Quat sQsv(vr[0],osg::Vec3d(1,0,0), vr[1],osg::Vec3d(0,1,0), vr[2],osg::Vec3d(0,0,1));
 transform->setAttitude(sQsv);
 //baseTransform->setAttitude(osg::Quat(js->pose.pose.orientation.w,osg::Vec3d(0,0,1)));
 }
 }

 ~ROSNavigationDataToPAT(){}
 };
 */

ROSJointStateToArm::ROSJointStateToArm(std::string topic, boost::shared_ptr<SimulatedIAUV> arm) :
    ROSSubscriberInterface(topic)
{
  this->arm = arm;
}

void ROSJointStateToArm::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSJointStateToArm subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < sensor_msgs::JointState > (topic, 10, &ROSJointStateToArm::processData, this);
}

void ROSJointStateToArm::processData(const sensor_msgs::JointState::ConstPtr& js)
{
  //Receive request from client
  if (js->position.size() != 0)
  {
    //position command
    std::vector < std::string > names = js->name;
    std::vector<double> position = js->position;
    arm->urdf->setJointPosition(position, names);
  }
  else if (js->velocity.size() != 0)
  {
    //velocity command
    std::vector < std::string > names = js->name;
    std::vector<double> velocity = js->velocity;
    arm->urdf->setJointVelocity(velocity, names);
  }
}

ROSJointStateToArm::~ROSJointStateToArm()
{
}

ROSImageToHUDCamera::ROSImageToHUDCamera(std::string topic, std::string info_topic, boost::shared_ptr<HUDCamera> camera) :
    ROSSubscriberInterface(info_topic), cam(camera), image_topic(topic)
{
}

void ROSImageToHUDCamera::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSImageToHUDCamera subscriber on topic %s", topic.c_str());
  it.reset(new image_transport::ImageTransport(nh));
  OSG_DEBUG << "ROSImageToHUDCamera::createSubscriber Subscribing to image topic " << image_topic << std::endl;
  image_sub = it->subscribe(image_topic, 1, &ROSImageToHUDCamera::processData, this);
  //OSG_INFO << "ROSCamera::ROSCamera Subscribing to camera info topic " << info_topic << std::endl;
  //sub_=nh_.subscribe<sensor_msgs::CameraInfo>(info_topic, 1, &ROSImageToHUDCamera::imageInfoCallback, this);
}

void ROSImageToHUDCamera::processData(const sensor_msgs::ImageConstPtr& msg)
{
  OSG_DEBUG << "ROSImageToHUDCamera::imageCallback start: " << msg->width << "x" << msg->height << " step:" << msg->step
      << std::endl;

  //unsigned char *msgdata=(unsigned char*)&(msg->data[0]);
  //osg_image->setImage(width,height,1,0,GL_RGB,GL_UNSIGNED_BYTE,msgdata,osg::Image::NO_DELETE);
  char *osgimage_data = (char*)cam->osg_image->data();
  //Memory cannot be directly copied, since the image frame used in OpenSceneGraph (OpenGL glReadPixels) is on
  //the bottom-left looking towards up-right, whereas ROS sensor_msgs::Image::data expects origin on top-left
  //looking towards bottom-right. Therefore it must be manually arranged, although this could be much improved:
  for (unsigned int i = 0; i < msg->height; i++)
  {
    for (unsigned int j = 0; j < msg->step; j++)
    {
      osgimage_data[i * msg->step + j] = msg->data[(msg->height - i - 1) * msg->step + j];
    }
  }
  cam->ready_ = true;
  OSG_DEBUG << "ROSImageToHUDCamera::imageCallback exit" << std::endl;
}

ROSImageToHUDCamera::~ROSImageToHUDCamera()
{
}

ROSPublisherInterface::ROSPublisherInterface(std::string topic, int publish_rate) :
    ROSInterface(topic)
{
  this->publish_rate = publish_rate;
  OSG_DEBUG << "ROSPublisherInterface Thread starting..." << topic << std::endl;
  startThread();
  OSG_DEBUG << "ROSPublisherInterface Thread created" << std::endl;
}

/* Thread code */
void ROSPublisherInterface::run()
{
  ros::Duration(2).sleep();
  createPublisher (nh_);

  ros::Rate rate(publish_rate);
  while (ros::ok())
  {
    publish();

    rate.sleep();
  }
}

ROSPublisherInterface::~ROSPublisherInterface()
{
  join();
}

PATToROSOdom::PATToROSOdom(osg::Group *rootNode, std::string vehicleName, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate)
{

  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
}

void PATToROSOdom::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("PATToROSOdom publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < nav_msgs::Odometry > (topic, 1);
}

void PATToROSOdom::publish()
{
  if (transform != NULL)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = getROSTime();

    osg::Matrixd mat = transform->getMatrix();
    osg::Vec3d pos = mat.getTrans();
    osg::Quat rot = mat.getRotate();

    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    odom.pose.pose.orientation.x = rot.x();
    odom.pose.pose.orientation.y = rot.y();
    odom.pose.pose.orientation.z = rot.z();
    odom.pose.pose.orientation.w = rot.w();

    //twist and covariances not implemented at the moment
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;
    for (int i = 0; i < 36; i++)
    {
      odom.twist.covariance[i] = 0;
      odom.pose.covariance[i] = 0;
    }

    pub_.publish(odom);
  }
}

PATToROSOdom::~PATToROSOdom()
{
}

ImuToROSImu::ImuToROSImu(InertialMeasurementUnit *imu, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), imu_(imu)
{
}

void ImuToROSImu::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("Imu publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::Imu > (topic, 1);
}

void ImuToROSImu::publish()
{
  if (imu_ != NULL)
  {
    osg::Quat rot = imu_->getMeasurement();

    sensor_msgs::Imu imu;
    imu.header.stamp = getROSTime();
    imu.header.frame_id = "world";
    imu.orientation.x = rot.x();
    imu.orientation.y = rot.y();
    imu.orientation.z = rot.z();
    imu.orientation.w = rot.w();

    imu.orientation_covariance[0] = imu.orientation_covariance[4] = imu.orientation_covariance[8] = std::pow(
        imu_->getStandardDeviation(), 2);

    pub_.publish(imu);
  }
}

ImuToROSImu::~ImuToROSImu()
{
}

PressureSensorToROS::PressureSensorToROS(PressureSensor *sensor, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), sensor_(sensor)
{
}

void PressureSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("PressureSensor publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < underwater_sensor_msgs::Pressure > (topic, 1);
}

void PressureSensorToROS::publish()
{
  if (sensor_ != NULL)
  {
    double pressure = sensor_->getMeasurement();

    underwater_sensor_msgs::Pressure v;
    v.pressure = pressure;
    v.header.stamp = getROSTime();
    v.header.frame_id = "world";

    pub_.publish(v);
  }
}

PressureSensorToROS::~PressureSensorToROS()
{
}

void GPSSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("GPSSensor publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::NavSatFix > (topic, 1);
}

void GPSSensorToROS::publish()
{
  if (sensor_ != NULL)
  {
    osg::Vec3d wTgps = sensor_->getMeasurement();

    //publish only if near to the ocean surface
    if (sensor_->depthBelowWater() < 0.5)
    {
      sensor_msgs::NavSatFix m;
      m.latitude = wTgps[0];
      m.longitude = wTgps[1];
      m.position_covariance[0] = m.position_covariance[4] = m.position_covariance[8] = std::pow(
          sensor_->getStandardDeviation(), 2);
      m.position_covariance_type = m.COVARIANCE_TYPE_DIAGONAL_KNOWN;

      pub_.publish(m);
    }
  }
}

void DVLSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("DVLSensor publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < underwater_sensor_msgs::DVL > (topic, 1);
}

void DVLSensorToROS::publish()
{
  if (sensor_ != NULL)
  {
    osg::Vec3d vdvl = sensor_->getMeasurement();

    underwater_sensor_msgs::DVL m;
    m.bi_x_axis = vdvl.x();
    m.bi_y_axis = vdvl.y();
    m.bi_z_axis = vdvl.z();

    pub_.publish(m);
  }
}

ArmToROSJointState::ArmToROSJointState(SimulatedIAUV *arm, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate)
{
  this->arm = arm->urdf;
}

void ArmToROSJointState::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("ArmToROSJointState publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::JointState > (topic, 1);
}

void ArmToROSJointState::publish()
{
  if (arm != NULL)
  {
    sensor_msgs::JointState js;
    js.header.stamp = getROSTime();
    std::vector<double> q = arm->getJointPosition();
    std::vector<std::string> names=arm->getJointName();
    for (size_t i = 0; i < q.size(); i++)
    {
      js.name.push_back(names[i]);
      js.position.push_back(q[i]);
      js.effort.push_back(0);
    }

    pub_.publish(js);
  }
}

ArmToROSJointState::~ArmToROSJointState()
{
}

VirtualCameraToROSImage::VirtualCameraToROSImage(VirtualCamera *camera, std::string topic, std::string info_topic,
                                                 int rate, int depth) :
    ROSPublisherInterface(info_topic, rate), cam(camera), image_topic(topic)
{
  it.reset(new image_transport::ImageTransport(nh_));
  this->depth = depth;
  this->bw = camera->bw;
}

void VirtualCameraToROSImage::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("VirtualCameraToROSImage publisher on topic %s", topic.c_str());
  while (!it)
  {
    ROS_INFO("VirtualCameraToROSImage Waiting for transport to be initialized...");
  }
  img_pub_ = it->advertise(image_topic, 1);
  pub_ = nh.advertise < sensor_msgs::CameraInfo > (topic, 1);
}

void VirtualCameraToROSImage::publish()
{
  //OSG_DEBUG << "OSGImageToROSImage::publish entering" << std::endl;
  osg::ref_ptr < osg::Image > osgimage;
  if (depth)
  {
    osgimage = cam->depthTexture;
  }
  else
  {
    osgimage = cam->renderTexture;
  }
  if (osgimage != NULL && osgimage->getTotalSizeInBytes() != 0)
  {
    //OSG_DEBUG << "\t image size: " << cam->renderTexture->s() << " " << cam->renderTexture->t() << " " << cam->renderTexture->getTotalSizeInBytes() << std::endl;
    int w, h, d;
    w = osgimage->s();
    h = osgimage->t();
    d = osgimage->getTotalSizeInBytes();

    if (d != 0)
    {
      sensor_msgs::Image img;
      sensor_msgs::CameraInfo img_info;
      img_info.header.stamp = img.header.stamp = getROSTime();
      img_info.header.frame_id = img.header.frame_id = cam->frameId;
      if (depth)
        img.encoding = std::string("mono16");
      else if (bw)
        img.encoding = std::string("mono8");
      else
        img.encoding = std::string("rgb8");

      img.is_bigendian = 0;
      img.height = h;
      img.width = w;
      img.step = d / h;
      img.data.resize(d);
      img_info.width = w;
      img_info.height = h;

      img_info.D.resize(4, 0.0);

      img_info.R[0] = 1.0;
      img_info.R[4] = 1.0;
      img_info.R[8] = 1.0;

      img_info.K[0] = cam->fx;
      img_info.K[2] = cam->cx;
      img_info.K[4] = cam->fy;
      img_info.K[5] = cam->cy;
      img_info.K[8] = 1;

      img_info.P[0] = cam->fx;
      img_info.P[2] = cam->cx;
      img_info.P[3] = cam->Tx;
      img_info.P[5] = cam->fy;
      img_info.P[6] = cam->cy;
      img_info.P[7] = cam->Ty;
      img_info.P[10] = 1;

      img_info.roi.x_offset = 0;
      img_info.roi.y_offset = 0;
      img_info.roi.height = img_info.height;
      img_info.roi.width = img_info.width;
      img_info.roi.do_rectify = false;

      img_info.binning_x = 0;
      img_info.binning_y = 0;

      //img_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

      unsigned char *virtualdata = (unsigned char*)osgimage->data();
      //memcpy(&(img.data.front()),virtualdata,d*sizeof(char));
      //Memory cannot be directly copied, since the image frame used in OpenSceneGraph (OpenGL glReadPixels) is on
      //the bottom-left looking towards up-right, whereas ROS sensor_msgs::Image::data expects origin on top-left
      //looking towards bottom-right. Therefore it must be manually arranged.
      if (virtualdata != NULL)
        for (int i = 0; i < h; i++)
        {
          for (unsigned int j = 0;
              (!bw && !depth && j < img.step) || (bw && j * 3 < img.step) || (depth && j * 2 < img.step); j++)
          {
            if (bw)
            {
              img.data[(h - i - 1) * img.step + j] = virtualdata[i * img.step + j * 3] * 0.2989;
              img.data[(h - i - 1) * img.step + j] += virtualdata[i * img.step + j * 3 + 1] * 0.5870;
              img.data[(h - i - 1) * img.step + j] += virtualdata[i * img.step + j * 3 + 2] * 0.1140;
            }
            else if (depth)
            {
              img.data[(h - i - 1) * img.step + j] =
                  j % 2 ? virtualdata[i * img.step + j * 2] : virtualdata[i * img.step + j * 2 + 1];
              //img.data[(h-i-1)*img.step+j]+=virtualdata[i*img.step+j*2+1];
            }
            else
              img.data[(h - i - 1) * img.step + j] = virtualdata[i * img.step + j];
          }
        }
      else
        memset(&(img.data.front()), 0, d);

      img_pub_.publish(img);
      pub_.publish(img_info);
    }
  }
  //OSG_DEBUG << "OSGImageToROSImage::publish exit" << std::endl;		
}

VirtualCameraToROSImage::~VirtualCameraToROSImage()
{
}

RangeSensorToROSRange::RangeSensorToROSRange(VirtualRangeSensor *rangesensor, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), rs(rangesensor)
{
}

void RangeSensorToROSRange::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("RangeSensorToROSRange publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::Range > (topic, 1);
}

void RangeSensorToROSRange::publish()
{
  if (rs != NULL)
  {
    sensor_msgs::Range r;
    r.header.stamp = getROSTime();
    r.radiation_type = sensor_msgs::Range::ULTRASOUND;
    r.field_of_view = 0; //X axis of the sensor
    r.min_range = 0;
    r.max_range = rs->range;
    r.range = (rs->node_tracker != NULL) ? rs->node_tracker->distance_to_obstacle : r.max_range;

    pub_.publish(r);
  }
}

RangeSensorToROSRange::~RangeSensorToROSRange()
{
}

MultibeamSensorToROS::MultibeamSensorToROS(MultibeamSensor *multibeamSensor, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), MB(multibeamSensor)
{
}

void MultibeamSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO(" MultibeamSensorToROS publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::LaserScan > (topic, 1);
}

void MultibeamSensorToROS::publish()
{
  if (MB != NULL)
  {
    sensor_msgs::LaserScan ls;
    ls.header.stamp = getROSTime();
    ls.header.frame_id = MB->name;

    double fov, aspect, near, far;

    MB->textureCamera->getProjectionMatrixAsPerspective(fov, aspect, near, far);
    ls.range_min = near;
    ls.range_max = MB->range; //far plane should be higher (z-buffer resolution)
    ls.angle_min = MB->initAngle * M_PI / 180;
    ls.angle_max = MB->finalAngle * M_PI / 180;
    ls.angle_increment = MB->angleIncr * M_PI / 180;
    ls.ranges.resize(MB->numpixels);
    std::vector<double> tmp;
    tmp.resize(MB->numpixels);

    float * data = (float *)MB->depthTexture->data();
    double a = far / (far - near);
    double b = (far * near) / (near - far);

    for (int i = 0; i < MB->numpixels; i++)
    {
      double Z = (data[i]); ///4294967296.0;
      tmp[i] = b / (Z - a);
      if (tmp[i] > MB->range)
        tmp[i] = MB->range;
    }
    for (int i = 0; i < MB->numpixels; i++)
    {
      ls.ranges[i] = (tmp[MB->remapVector[i].pixel1] * MB->remapVector[i].weight1
          + tmp[MB->remapVector[i].pixel2] * MB->remapVector[i].weight2) * MB->remapVector[i].distort;
    }

    /*r.radiation_type=sensor_msgs::Range::ULTRASOUND;
     r.field_of_view=0;	//X axis of the sensor
     r.min_range=0;
     r.max_range=rs->range;
     r.range= (rs->node_tracker!=NULL) ? rs->node_tracker->distance_to_obstacle : r.max_range;*/
    pub_.publish(ls);
  }
}

MultibeamSensorToROS::~MultibeamSensorToROS()
{
}

contactSensorToROS::contactSensorToROS(osg::Group *rootNode, BulletPhysics * physics, std::string target,
                                       std::string topic, int rate) :
    ROSPublisherInterface(topic, rate)
{
  this->rootNode = rootNode;
  this->physics = physics;
  this->target = target;
}

void contactSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("contactSensorToROS publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < std_msgs::Bool > (topic, 1);
}

void contactSensorToROS::publish()
{
  int colliding = 0;

  for (int i = 0; i < physics->getNumCollisions(); i++)
  {
    btPersistentManifold * col = physics->getCollision(i);

    //Get objects colliding
    btRigidBody* obA = static_cast<btRigidBody*>(col->getBody0());
    btRigidBody* obB = static_cast<btRigidBody*>(col->getBody1());

    //Check if target is involved in collision
    CollisionDataType * data = (CollisionDataType *)obA->getUserPointer();
    CollisionDataType * data2 = (CollisionDataType *)obB->getUserPointer();

    int numContacts = col->getNumContacts();

    if (data2->getObjectName() == target || data->getObjectName() == target)
    {
      for (int j = 0; j < numContacts; j++)
      {
        btManifoldPoint pt = col->getContactPoint(j);
        if (pt.getDistance() < 0.f)
          colliding = 1;
      }
    }

  }
  std_msgs::Bool msg;
  msg.data = colliding;
  pub_.publish(msg);
}

contactSensorToROS::~contactSensorToROS()
{
}


WorldToROSTF::WorldToROSTF(osg::Group *rootNode,  std::vector< boost::shared_ptr<SimulatedIAUV> > iauvFile, std::string worldRootName, unsigned int enableObjects, int rate ) :
    ROSPublisherInterface(worldRootName, rate)
{
   iauvFile_ = iauvFile;
   for(int i = 0; i < iauvFile_.size(); i++)
   {
      KDL::Tree tree;
      if (!kdl_parser::treeFromFile(iauvFile[i].get()->urdf->URDFFile, tree))
      {
         ROS_ERROR("Failed to construct kdl tree");
      }
      else
      {
         ROS_INFO("Loaded tree, %d segments, %d joints", tree.getNrOfSegments(), tree.getNrOfJoints());
      }
      
      osg::ref_ptr<osg::MatrixTransform> transform;
      robot_pubs_.push_back(boost::shared_ptr<robot_state_publisher::RobotStatePublisher>(
       new robot_state_publisher::RobotStatePublisher(tree)));
  
      findNodeVisitor findNode(iauvFile[i].get()->name);
      rootNode->accept(findNode);
      osg::Node *first = findNode.getFirst();
      if (first == NULL)
      {
         transform = NULL;
      }
      else
      {
         transform = dynamic_cast<osg::MatrixTransform*>(first);
      }
      transforms_.push_back(transform);
   }
   worldRootName_ = worldRootName;
   enableObjects_ = enableObjects;
}

void WorldToROSTF::createPublisher(ros::NodeHandle &nh)
{   
   odompub_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
}

void WorldToROSTF::publish()
{
   for( int i = 0; i < iauvFile_.size(); i++ )
   {
      std::vector<double> q = iauvFile_[i].get()->urdf->getJointPosition();
      std::vector<std::string> names= iauvFile_[i].get()->urdf->getJointName();
      std::map<std::string, double> js;
      assert(names.size() == q.size());
      for (size_t j = 0; j < names.size(); ++j)
         js[names[j]] = q[j];
  
     // Publish moving joints
     robot_pubs_[i]->RobotStatePublisher::publishTransforms(js, getROSTime(), iauvFile_[i].get()->name);
     // Publish fixed joints
     robot_pubs_[i]->RobotStatePublisher::publishFixedTransforms(iauvFile_[i].get()->name);
     //Publish odometry
      if (transforms_[i] != NULL)
      {
         osg::Matrixd mat = transforms_[i]->getMatrix();
         osg::Vec3d pos = mat.getTrans();
         osg::Quat rot = mat.getRotate();

         tf::Vector3 p(pos.x(), pos.y(), pos.z());
         tf::Quaternion q(rot.x(), rot.y(), rot.z(), rot.w());
         tf::Pose pose(q,p);
         tf::StampedTransform t(pose, getROSTime(), "/" + worldRootName_, "/"+iauvFile_[i].get()->name);
      
         tf::Vector3 p2(0, 0, 0);
         tf::Quaternion q2(0, 0, 0, 1);
         tf::Pose pose2(q2,p2);
         tf::StampedTransform t2(pose2, getROSTime(), "/"+iauvFile_[i].get()->name, "/"+iauvFile_[i].get()->name + "/base_link");

         odompub_->sendTransform(t2);
         odompub_->sendTransform(t);  
      }
   }
}

WorldToROSTF::~WorldToROSTF()
{
}
