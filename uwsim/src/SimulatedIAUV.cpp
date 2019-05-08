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

#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <uwsim/NED.h>
#include <uwsim/SceneBuilder.h>
#include <uwsim/SimulatedIAUV.h>
#include <uwsim/SimulatorConfig.h>
#include <uwsim/URDFRobot.h>
#include <uwsim/UWSimUtils.h>
#include <uwsim/osgOceanScene.h>

/** Callback for updating the vehicle lamp according to the vehicle position */
/*
 class LightUpdateCallback:public osg::NodeCallback {
 osg::Transform *trackNode;	///< Node that the light must track

 public:
 LightUpdateCallback(osg::Transform *trackNode)
 {this->trackNode=trackNode;}

 void operator () (osg::Node *node, osg::NodeVisitor *nv) {
 //update light position to track the node
 osg::LightSource *ls=dynamic_cast<osg::LightSource*>(node);
 osg::Light *l=ls->getLight();
 osg::PositionAttitudeTransform *pat=trackNode->asPositionAttitudeTransform();
 osg::Vec3d pos=pat->getPosition();
 l->setPosition( osg::Vec4f(pos.x(),pos.y(),pos.z()-0.5, 1.f) );

 //call to standard callback
 osg::NodeCallback::operator()(node,nv);
 }
 };
 */

/*
 SimulatedIAUV::SimulatedIAUV(osgOcean::OceanScene *oscene, arm_t armtype) {
 vehicle=new SimulatedVehicle(oscene, "GIRONA500/girona500.osg");

 if (armtype==PA10)
 arm=new SimulatedPA10(oscene);
 else if (armtype==ARM5)
 arm=new SimulatedArmFromURDF5(oscene);

 baseTransform=NULL;
 if(vehicle->baseTransform!=NULL && arm->baseTransform!=NULL) {
 baseTransform=vehicle->baseTransform;
 baseTransform->addChild(arm->baseTransform);

 //Vehicle frame to Arm base frame transform
 osg::Matrix m=arm->baseTransform->getMatrix();
 if (armtype==PA10) {
 m.makeRotate(M_PI,1,0,0);
 } else if (armtype==ARM5) {
 }
 arm->baseTransform->setMatrix(m);
 }
 camview=NULL;

 //Set-up a lamp attached to the vehicle
 osg::Light *_light=new osg::Light;
 _light->setLightNum(1);
 _light->setAmbient( osg::Vec4d(1.0f, 1.0f, 1.0f, 1.0f ));
 _light->setDiffuse( osg::Vec4d( 1.0, 1.0, 1.0, 1.0 ) );
 _light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
 _light->setDirection(osg::Vec3d(0.0, 0.0, -5.0));
 _light->setSpotCutoff(40.0);
 _light->setSpotExponent(10.0);

 lightSource = new osg::LightSource;
 lightSource->setLight(_light);
 lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
 lightSource->setUpdateCallback(new LightUpdateCallback(baseTransform));
 }
 */

SimulatedIAUV::SimulatedIAUV(SceneBuilder *oscene, Vehicle vehicleChars)
    : urdf(new URDFRobot(oscene->scene->getOceanScene(), vehicleChars)) {
  name = vehicleChars.name;
  baseTransform = new osg::MatrixTransform;
  root = oscene->root;
  if (vehicleChars.vr)
    baseTransform->setNodeMask(oscene->scene->getOceanScene()->getARMask());

  if (urdf->baseTransform != NULL /* && arm->baseTransform!=NULL*/) {
    baseTransform->addChild(urdf->baseTransform);
    baseTransform->setName(vehicleChars.name);
  }
  scale = osg::Vec3d(vehicleChars.scale[0], vehicleChars.scale[1],
                     vehicleChars.scale[2]);

  // Add virtual  cameras in config file
  while (vehicleChars.Vcams.size() > 0) {
    Vcam vcam = vehicleChars.Vcams.front();
    OSG_INFO << "Adding a virtual camera " << vcam.name << "..." << std::endl;
    vehicleChars.Vcams.pop_front();
    // Camera frame given wrt vehicle origin frame.
    // Remember that in opengl/osg, the camera frame is a right-handed system
    // with Z going backwards (opposite to the viewing direction) and Y up.
    osg::ref_ptr<osg::Transform> vMc =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMc->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(vcam.position[0], vcam.position[1], vcam.position[2]));
    vMc->asPositionAttitudeTransform()->setAttitude(osg::Quat(
        vcam.orientation[0], osg::Vec3d(1, 0, 0), vcam.orientation[1],
        osg::Vec3d(0, 1, 0), vcam.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[vcam.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMc);
    camview.push_back(VirtualCamera(oscene->root, vcam.name, vcam.linkName, vMc,
                                    vcam.resw, vcam.resh, vcam.baseLine,
                                    vcam.frameId, vcam.fov, oscene, vcam.std,
                                    vcam.parameters.get(), 0, vcam.bw));
    if (vcam.showpath)
      camview[camview.size() - 1].showPath(vcam.showpath);
    OSG_INFO << "Done adding a virtual camera..." << std::endl;
  }

  // Add virtual range cameras in config file
  while (vehicleChars.VRangecams.size() > 0) {
    Vcam vcam = vehicleChars.VRangecams.front();
    OSG_INFO << "Adding a virtual camera " << vcam.name << "..." << std::endl;
    vehicleChars.VRangecams.pop_front();
    // Camera frame given wrt vehicle origin frame.
    // Remember that in opengl/osg, the camera frame is a right-handed system
    // with Z going backwards (opposite to the viewing direction) and Y up.
    osg::ref_ptr<osg::Transform> vMc =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMc->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(vcam.position[0], vcam.position[1], vcam.position[2]));
    vMc->asPositionAttitudeTransform()->setAttitude(osg::Quat(
        vcam.orientation[0], osg::Vec3d(1, 0, 0), vcam.orientation[1],
        osg::Vec3d(0, 1, 0), vcam.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[vcam.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMc);
    camview.push_back(VirtualCamera(oscene->root, vcam.name, vcam.linkName, vMc,
                                    vcam.resw, vcam.resh, vcam.baseLine,
                                    vcam.frameId, vcam.fov, NULL, 0,
                                    vcam.parameters.get(), 1, 0));
    if (vcam.showpath)
      camview[camview.size() - 1].showPath(vcam.showpath);
    // Check underwaterParticles to change the mask
    if (!vcam.underwaterParticles)
      camview[camview.size() - 1].textureCamera->setCullMask(
          oscene->scene->getOceanScene()->getNormalSceneMask());
    OSG_INFO << "Done adding a virtual camera..." << std::endl;
  }

  // Adding Structured light projector
  while (vehicleChars.sls_projectors.size() > 0) {
    OSG_INFO << "Adding a structured light projector..." << std::endl;
    slProjector slp;
    slp = vehicleChars.sls_projectors.front();
    vehicleChars.sls_projectors.pop_front();
    osg::ref_ptr<osg::Transform> vMp =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMp->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(slp.position[0], slp.position[1], slp.position[2]));
    vMp->asPositionAttitudeTransform()->setAttitude(osg::Quat(
        slp.orientation[0], osg::Vec3d(1, 0, 0), slp.orientation[1],
        osg::Vec3d(0, 1, 0), slp.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[slp.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMp);
    // camview.push_back(VirtualCamera(oscene->root, "slp_camera", vMp, 512,
    // 512,slp.fov,102.4));
    sls_projectors.push_back(VirtualSLSProjector(
        slp.name, slp.linkName,
        oscene->root, // maybe oscene->scene->localizedWorld ?
        vMp, slp.image_name, slp.fov, (slp.laser) ? true : false));
    camview.push_back(sls_projectors.back().camera);
    OSG_INFO << "Done adding a structured light projector..." << std::endl;
  }

  // Adding range sensors
  while (vehicleChars.range_sensors.size() > 0) {
    OSG_INFO << "Adding a virtual range sensor..." << std::endl;
    rangeSensor rs;
    rs = vehicleChars.range_sensors.front();
    vehicleChars.range_sensors.pop_front();
    osg::ref_ptr<osg::Transform> vMr =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMr->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(rs.position[0], rs.position[1], rs.position[2]));
    vMr->asPositionAttitudeTransform()->setAttitude(
        osg::Quat(rs.orientation[0], osg::Vec3d(1, 0, 0), rs.orientation[1],
                  osg::Vec3d(0, 1, 0), rs.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[rs.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMr);
    range_sensors.push_back(
        VirtualRangeSensor(rs.name, rs.linkName, oscene->scene->localizedWorld,
                           vMr, rs.range, (rs.visible) ? true : false,
                           oscene->scene->getOceanScene()->getARMask()));
    OSG_INFO << "Done adding a virtual range sensor..." << std::endl;
  }

  // Adding imus
  while (vehicleChars.imus.size() > 0) {
    OSG_INFO << "Adding an IMU..." << std::endl;
    Imu imu;
    imu = vehicleChars.imus.front();
    vehicleChars.imus.pop_front();
    osg::ref_ptr<osg::Transform> vMi =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMi->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(imu.position[0], imu.position[1], imu.position[2]));
    vMi->asPositionAttitudeTransform()->setAttitude(osg::Quat(
        imu.orientation[0], osg::Vec3d(1, 0, 0), imu.orientation[1],
        osg::Vec3d(0, 1, 0), imu.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[imu.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMi);
    imus.push_back(InertialMeasurementUnit(
        imu.name, imu.linkName, vMi, oscene->scene->localizedWorld->getMatrix(),
        imu.std));
    OSG_INFO << "Done adding an IMU..." << std::endl;
  }

  // Adding pressure sensors
  while (vehicleChars.pressure_sensors.size() > 0) {
    OSG_INFO << "Adding a pressure sensor..." << std::endl;
    XMLPressureSensor ps;
    ps = vehicleChars.pressure_sensors.front();
    vehicleChars.pressure_sensors.pop_front();
    osg::ref_ptr<osg::Transform> vMs =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMs->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(ps.position[0], ps.position[1], ps.position[2]));
    vMs->asPositionAttitudeTransform()->setAttitude(
        osg::Quat(ps.orientation[0], osg::Vec3d(1, 0, 0), ps.orientation[1],
                  osg::Vec3d(0, 1, 0), ps.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[ps.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMs);
    pressure_sensors.push_back(
        PressureSensor(ps.name, ps.linkName, vMs,
                       oscene->scene->localizedWorld->getMatrix(), ps.std));
    OSG_INFO << "Done adding an Pressure Sensor..." << std::endl;
  }

  // Adding GPS sensors
  while (vehicleChars.gps_sensors.size() > 0) {
    OSG_INFO << "Adding a gps sensor..." << std::endl;
    XMLGPSSensor ps;
    ps = vehicleChars.gps_sensors.front();
    vehicleChars.gps_sensors.pop_front();
    osg::ref_ptr<osg::Transform> vMs =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMs->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(ps.position[0], ps.position[1], ps.position[2]));
    vMs->asPositionAttitudeTransform()->setAttitude(
        osg::Quat(ps.orientation[0], osg::Vec3d(1, 0, 0), ps.orientation[1],
                  osg::Vec3d(0, 1, 0), ps.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[ps.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMs);
    gps_sensors.push_back(GPSSensor(oscene->scene, ps.name, ps.linkName, vMs,
                                    oscene->scene->localizedWorld->getMatrix(),
                                    ps.std));
    OSG_INFO << "Done adding an GPS Sensor..." << std::endl;
  }

  // Adding dvl sensors
  while (vehicleChars.dvl_sensors.size() > 0) {
    OSG_INFO << "Adding a dvl sensor..." << std::endl;
    XMLDVLSensor ps;
    ps = vehicleChars.dvl_sensors.front();
    vehicleChars.dvl_sensors.pop_front();
    osg::ref_ptr<osg::Transform> vMs =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMs->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(ps.position[0], ps.position[1], ps.position[2]));
    vMs->asPositionAttitudeTransform()->setAttitude(
        osg::Quat(ps.orientation[0], osg::Vec3d(1, 0, 0), ps.orientation[1],
                  osg::Vec3d(0, 1, 0), ps.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[ps.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMs);
    dvl_sensors.push_back(DVLSensor(ps.name, ps.linkName, vMs,
                                    oscene->scene->localizedWorld->getMatrix(),
                                    ps.std));
    OSG_INFO << "Done adding an DVL Sensor..." << std::endl;
  }

  // Adding Multibeam sensors
  while (vehicleChars.multibeam_sensors.size() > 0) {
    OSG_INFO << "Adding a Multibeam sensor..." << std::endl;
    XMLMultibeamSensor MB;
    MB = vehicleChars.multibeam_sensors.front();
    vehicleChars.multibeam_sensors.pop_front();
    osg::ref_ptr<osg::Transform> vMs =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMs->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(MB.position[0], MB.position[1], MB.position[2]));
    vMs->asPositionAttitudeTransform()->setAttitude(
        osg::Quat(MB.orientation[0], osg::Vec3d(1, 0, 0), MB.orientation[1],
                  osg::Vec3d(0, 1, 0), MB.orientation[2], osg::Vec3d(0, 0, 1)));
    urdf->link[MB.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMs);
    unsigned int mask;
    if (MB.underwaterParticles)
      mask = oscene->scene->getOceanScene()->getARMask();
    else
      mask =
          oscene->scene->getOceanScene()->getNormalSceneMask(); // Normal Scene
                                                                // mask should
                                                                // be enough for
                                                                // range sensor
    MultibeamSensor mb = MultibeamSensor(
        oscene->root, MB.name, MB.linkName, vMs, MB.initAngle, MB.finalAngle,
        MB.angleIncr, MB.range, mask, MB.visible, mask);
    multibeam_sensors.push_back(mb);
    for (unsigned int i = 0; i < mb.nCams; i++)
      camview.push_back(mb.vcams[i]);
    OSG_INFO << "Done adding a Multibeam Sensor..." << std::endl;
  }

  // Adding object pickers
  while (vehicleChars.object_pickers.size() > 0) {
    OSG_INFO << "Adding an object picker..." << std::endl;
    rangeSensor rs;
    rs = vehicleChars.object_pickers.front();
    vehicleChars.object_pickers.pop_front();
    osg::ref_ptr<osg::Transform> vMr =
        (osg::Transform *)new osg::PositionAttitudeTransform;
    vMr->asPositionAttitudeTransform()->setPosition(
        osg::Vec3d(rs.position[0], rs.position[1], rs.position[2]));
    vMr->asPositionAttitudeTransform()->setAttitude(
        osg::Quat(rs.orientation[0], osg::Vec3d(1, 0, 0), rs.orientation[1],
                  osg::Vec3d(0, 1, 0), rs.orientation[2], osg::Vec3d(0, 0, 1)));
    vMr->setName("ObjectPickerNode");
    urdf->link[rs.link]->asGroup()->addChild(vMr);
    object_pickers.push_back(
        ObjectPicker(rs.name, oscene->scene->localizedWorld, vMr, rs.range,
                     true, urdf, oscene->scene->getOceanScene()->getARMask()));
    OSG_INFO << "Done adding an object picker..." << std::endl;
  }

  devices.reset(new SimulatedDevices());
  devices->applyConfig(this, vehicleChars, oscene);

  // Set-up a lamp attached to the vehicle: TODO
  /*
   osg::Light *_light=new osg::Light;
   _light->setLightNum(1);
   _light->setAmbient( osg::Vec4d(1.0f, 1.0f, 1.0f, 1.0f ));
   _light->setDiffuse( osg::Vec4d( 1.0, 1.0, 1.0, 1.0 ) );
   _light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
   _light->setDirection(osg::Vec3d(0.0, 0.0, -5.0));
   _light->setSpotCutoff(40.0);
   _light->setSpotExponent(10.0);

   lightSource = new osg::LightSource;
   lightSource->setLight(_light);
   lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
   lightSource->setUpdateCallback(new LightUpdateCallback(baseTransform));
   */

  // Check and launch TF relative
  if (vehicleChars.tfRelativeConfig.tfId != "") {
    // launch TF relative worker
    std::thread worker([vehicleChars, this]() {
      tf::TransformListener wMoListener;
      tf::StampedTransform wMo;
      tf::Vector3 oTv, wTv;
      tf::Quaternion oRv, wRv;
      oTv.setX(vehicleChars.tfRelativeConfig.position[0]);
      oTv.setY(vehicleChars.tfRelativeConfig.position[1]);
      oTv.setZ(vehicleChars.tfRelativeConfig.position[2]);
      oRv.setRPY(vehicleChars.tfRelativeConfig.orientation[0],
                 vehicleChars.tfRelativeConfig.orientation[1],
                 vehicleChars.tfRelativeConfig.orientation[2]);

      oRv = oRv.normalize();
      tf::Transform oMv;
      oMv.setOrigin(oTv);
      oMv.setRotation(oRv);
      tf::Transform vehicleTf;
      std::string tfId = vehicleChars.tfRelativeConfig.tfId;
      tf::Matrix3x3 tmpMat;
      // vpHomogeneousMatrix sMsv;
      osg::Matrixd sMsv_osg;
      double roll, pitch, yaw;

      while (1) {
        try {
          wMoListener.waitForTransform("world", tfId, ros::Time(0),
                                       ros::Duration(1));
          wMoListener.lookupTransform("world", tfId, ros::Time(0), wMo);
          vehicleTf = wMo * oMv;

          wTv = vehicleTf.getOrigin();
          wRv = vehicleTf.getRotation();

          sMsv_osg.setTrans(wTv.x(), wTv.y(), wTv.z());
          sMsv_osg.setRotate(osg::Quat(wRv.x(), wRv.y(), wRv.z(), wRv.w()));
          sMsv_osg.preMultScale(baseTransform->getMatrix().getScale());

          setVehiclePosition(sMsv_osg);

        } catch (tf::TransformException &e) {
          ROS_WARN("TF relative worker: Not able to lookup transform: %s",
                   e.what());
        };
      }
    });
    worker.detach();
  }

  // Check and launch FDM
  if (vehicleChars.fdmPort != -1) {
    // launch FDM worker
    std::thread worker([vehicleChars, this]() {
      double degRad = 180 / M_PI;
      FDMData fdmData;
      int GCS_BUFFER_LENGTH = 4096;
      uint16_t localPort = vehicleChars.fdmPort;
      bool worldMned_set = false;
      tf::StampedTransform worldMned, nedMv;
      tf::Transform wMv;
      tf::Vector3 nedTv, vehicle_pos;
      tf::Quaternion nedRv, enuRv, vehicle_rot;
      tfScalar tmpMatVec[9];
      osg::Matrixd S, transform;
      int sockfd;
      struct sockaddr_in simAddr;
      struct sockaddr_in locAddr;
      sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
      memset(&locAddr, 0, sizeof(locAddr));
      locAddr.sin_family = AF_INET;
      locAddr.sin_addr.s_addr = INADDR_ANY;
      locAddr.sin_port = htons(localPort);
      tf::TransformListener wMnListener;
      if (fcntl(sockfd, F_SETFL, O_NONBLOCK) < 0) {
        close(sockfd);
        exit(EXIT_FAILURE);
      }
      if (-1 ==
          bind(sockfd, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
        perror("error bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
      }

      // ros::Rate rate(30);
      uint8_t rxbuff[GCS_BUFFER_LENGTH];

      uint32_t *version = (uint32_t *)rxbuff;
      uint32_t *padding = version + 1;
      double *longitude = (double *)(padding + 1);
      double *latitude = longitude + 1;
      double *altitude = latitude + 1;
      float *agl = (float *)(altitude + 1);
      float *phi = agl + 1;
      float *theta = phi + 1;
      float *psi = theta + 1;
      while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        memset(rxbuff, 0, GCS_BUFFER_LENGTH);
        socklen_t fromlen = sizeof(sockaddr);

        ssize_t recsize;
        do {
          recsize = recvfrom(sockfd, (void *)rxbuff, GCS_BUFFER_LENGTH, 0,
                             (struct sockaddr *)&simAddr, &fromlen);
        } while (recsize > 0);

        do {
          recsize = recvfrom(sockfd, (void *)rxbuff, GCS_BUFFER_LENGTH, 0,
                             (struct sockaddr *)&simAddr, &fromlen);
        } while (recsize <= 0);

        if (recsize > 0) {
          switch4Bytes(&fdmData.version, version);
          switch4Bytes(&fdmData.padding, padding);
          switch8Bytes(&fdmData.longitude, longitude);
          switch8Bytes(&fdmData.latitude, latitude);
          switch8Bytes(&fdmData.altitude, altitude);
          switch4Bytes(&fdmData.agl, agl);
          switch4Bytes(&fdmData.phi, phi);
          switch4Bytes(&fdmData.theta, theta);
          switch4Bytes(&fdmData.psi, psi);

          double ned_x, ned_y, ned_z;
          double lat, lon, alt;
          float roll, pitch, yaw;
          double yawDeg;

          lat = degRad * fdmData.latitude;
          lon = degRad * fdmData.longitude;
          alt = fdmData.altitude;
          roll = fdmData.phi;
          pitch = fdmData.theta;
          yaw = fdmData.psi;

          yawDeg = degRad * yaw;
          uwsim::NED::GetNED(lat, lon, alt, ned_x, ned_y, ned_z);
          // ROS_INFO("LL: %.9f, %.9f, %.9f ; NED: %f, %f, %f (Yaw: %f)", lat,
          // lon, alt, ned_x, ned_y, ned_z, yawDeg);
          // std::cout << std::flush;

          // Wait for transform
          if (worldMned_set) {
            nedTv.setX(ned_x);
            nedTv.setY(ned_y);
            nedTv.setZ(ned_z);
            // https://github.com/mavlink/mavros/issues/216
            nedRv.setRPY(roll, pitch, yaw);
            nedMv.setOrigin(nedTv);
            nedMv.setRotation(nedRv);
            wMv = worldMned * nedMv;

            S.makeScale(scale);
            wMv.getOpenGLMatrix(tmpMatVec);
            transform.set(tmpMatVec);
            transform = S * transform;
            setVehiclePosition(transform);
          } else {
            try {
              wMnListener.waitForTransform("world", "local_origin_ned",
                                           ros::Time(0), ros::Duration(1));
              wMnListener.lookupTransform("world", "local_origin_ned",
                                          ros::Time(0), worldMned);
              worldMned_set = true;
            } catch (tf::TransformException &e) {
              ROS_ERROR("Not able to lookup transform: %s", e.what());
              worldMned_set = false;
            };
          }
        }
      }
    });
    worker.detach();
  }
}

/*
 void SimulatedIAUV::setVirtualCamera(std::string name, osg::Transform*
 transform, int width, int height) {
 //Set I-AUV virtual camera

 baseTransform->asGroup()->addChild(transform);

 if (camview==NULL)
 camview=new VirtualCamera(name, transform, width, height);
 }
 */

/** Sets the vehicle position. (x,y,z) given wrt to the world frame.
 * (roll,pitch,yaw) are RPY angles in the local frame */
void SimulatedIAUV::setVehiclePosition(double x, double y, double z,
                                       double roll, double pitch, double yaw) {
  osg::Matrixd S, T, Rx, Ry, Rz, transform;
  T.makeTranslate(x, y, z);
  Rx.makeRotate(roll, 1, 0, 0);
  Ry.makeRotate(pitch, 0, 1, 0);
  Rz.makeRotate(yaw, 0, 0, 1);
  S.makeScale(scale);
  transform = S * Rz * Ry * Rx * T;
  setVehiclePosition(transform);
}

void SimulatedIAUV::setVehiclePosition(osg::Matrixd m) {
  baseTransform_mutex.lock();
  baseTransform->setMatrix(m);
  baseTransform_mutex.unlock();
}
