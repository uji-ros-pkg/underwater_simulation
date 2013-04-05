#include "MultibeamSensor.h"


MultibeamSensor::MultibeamSensor(osg::Group *uwsim_root, std::string name, osg::Node *trackNode,  double initAngle,double finalAngle,double alpha,double range):
  VirtualCamera(uwsim_root,name,trackNode,fabs(finalAngle-initAngle)/alpha+1,fabs(finalAngle-initAngle),range){

  this->numpixels=fabs(finalAngle-initAngle)/alpha+1;
  this->range=range;
  this->initAngle=initAngle;
  this->finalAngle=finalAngle;
  this->angleIncr=alpha;
  preCalcTable();
};

void MultibeamSensor::preCalcTable(){
  
  //Create matrix to unproject camera points to real world
  osg::Matrix *MVPW=new osg::Matrix(textureCamera->getViewMatrix() * textureCamera->getProjectionMatrix() * textureCamera->getViewport()->computeWindowMatrix());
  MVPW->invert(*MVPW);

  //Get real fov from camera
  osg::Vec3d first=osg::Vec3d(0,0,1)*(*MVPW), last=osg::Vec3d(numpixels-1,0,1)*(*MVPW), center=osg::Vec3d(numpixels/2,0,1)*(*MVPW);
  double realfov=acos((first*last)/(last.length()*first.length()));
  double thetacenter=acos((first*center)/(center.length()*first.length()));
  double alpha=realfov/(numpixels);
  //std::cout<<realfov<<" "<<alpha<<std::endl;


  //Interpolate points
  remapVector.resize(numpixels);
  int current=0;
  double lastTheta=0;
  for(int i=0;i<numpixels;i++){
    osg::Vec3d point=osg::Vec3d(i,0,1)*(*MVPW);

    double theta=acos((first*point) / (first.length()*point.length()));
    while(theta>=alpha*current && current<numpixels){
      if(theta==alpha*current){ //usually only first iteration as point has to be exactly the same
	remapVector[current].pixel1=i;
	remapVector[current].weight1=0.50;
	remapVector[current].pixel2=i;
	remapVector[current].weight2=0.50;
      }
      else{  //Interpolate between this and last point
        double dist=fabs(theta-alpha*current), prevdist=fabs(lastTheta-alpha*current);
	remapVector[current].pixel1=i;
	remapVector[current].weight1=prevdist/(dist+prevdist);
	remapVector[current].pixel2=i-1;
	remapVector[current].weight2=dist/(dist+prevdist);
	//std::cout<<remapVector[current].weight1<<" "<<remapVector[current].weight2<<" "<<remapVector[current].weight1+remapVector[current].weight2<<std::endl;
      }
      remapVector[current].distort=1+fabs(theta-thetacenter)*fabs(theta-thetacenter)*fabs(theta-thetacenter)/1.3;
      current++;
      //std::cout<<theta<<":"<<tan(theta)<<" asd:"<<fx<<std::endl;
      //std::cout<<current<<" "<<i<<std::endl;
    }
    lastTheta=theta;
    //std::cout<<" THETA: "<<theta<<"Current point: "<<current*alpha<<"Error: "<<theta-i*alpha<<"asd: "<<current<<std::endl;
  }


}
