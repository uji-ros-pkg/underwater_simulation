/*
 * GraspSpecification.cpp
 *
 *  Created on: 25/06/2012
 *      Author: toni
 */
#include "GraspSpecification.h"
#include <mar_robot_arm5e/ARM5Arm.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <tf/transform_datatypes.h>

GraspSpecification::GraspSpecification(): stop_(false) {
	feedback_sub=nh_.subscribe<visualization_msgs::InteractiveMarkerFeedback>("/basic_controls/feedback",1,&GraspSpecification::ReadFeedbackCallback, this);
}

GraspSpecification::~GraspSpecification(){
	stop_=true;
	join();
}
void GraspSpecification::ReadFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg){
	lastFeedback=msg;
}
void GraspSpecification::newPath(std::vector<double> offsetp, std::vector<double> offsetr, SceneBuilder *sceneBuilder){
	offsetp_=offsetp;
	offsetr_=offsetr;
	scene=sceneBuilder;
	cout<<"Posicion Final: "<<lastFeedback->pose.position.x<<" "<<lastFeedback->pose.position.y<<" "<<lastFeedback->pose.position.z<<std::endl;
	if(this->isRunning()){
		stop_=true;
		join();
	}
	startThread();
}
void GraspSpecification::run(){
	ARM5Arm arm;
	btQuaternion q;
	double roll, pitch, yaw;
	vpColVector finalPose;
	tf::quaternionMsgToTF(lastFeedback->pose.orientation, q);
	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	vpRxyzVector wVi(roll,pitch,yaw);
	vpRotationMatrix wRi(wVi);
	vpTranslationVector wTi(lastFeedback->pose.position.x, lastFeedback->pose.position.y, lastFeedback->pose.position.z);
	vpHomogeneousMatrix wMi(wTi, wRi);
	vpRxyzVector wVr(offsetr_[0], offsetr_[1], offsetr_[2]);
	vpRotationMatrix wRl(wVr);
	vpTranslationVector wTl(offsetp_[0], offsetp_[1], offsetp_[2]);
	vpHomogeneousMatrix wMl(wTl, wRl);
	vpHomogeneousMatrix lMi=wMl.inverse()*wMi;

	vpColVector colVector=arm.vehicleArmIK(lMi);
	vpHomogeneousMatrix hom=arm.directKinematics(colVector);
	finalPose.resize(colVector.getRows()+2);
	int aux=0;
	for(int i=0;i<colVector.getRows();i++){
		if(i==3){
			finalPose[3]=0;
			finalPose[4]=0;
			aux=2;
		}
		finalPose[i+aux]=colVector[i];
	}




	vpColVector actualPose, incrementPose, maxVelocities;
	actualPose.resize(colVector.getRows()+2);
	incrementPose.resize(colVector.getRows()+2);
	maxVelocities.resize(colVector.getRows()+2);

	maxVelocities[0]=0.00001;
	maxVelocities[1]=0.00001;
	maxVelocities[2]=0.00001;
	maxVelocities[3]=0.00001;
	maxVelocities[4]=0.00001;
	maxVelocities[5]=0.00001;
	maxVelocities[6]=0.00001;
	maxVelocities[7]=0.00001;
	maxVelocities[8]=0.00001;
	maxVelocities[9]=0.00001;



	std::vector<double> jointPositions=scene->iauvFile[0]->urdf->getJointPosition();
	actualPose[0]=scene->iauvFile[0]->baseTransform->getMatrix().getTrans().x();
	actualPose[1]=scene->iauvFile[0]->baseTransform->getMatrix().getTrans().y();
	actualPose[2]=scene->iauvFile[0]->baseTransform->getMatrix().getTrans().z();

	tf::Quaternion quat(scene->iauvFile[0]->baseTransform->getMatrix().getRotate().x(),scene->iauvFile[0]->baseTransform->getMatrix().getRotate().y(),scene->iauvFile[0]->baseTransform->getMatrix().getRotate().z(),scene->iauvFile[0]->baseTransform->getMatrix().getRotate().w());
	btMatrix3x3(quat).getRPY(roll, pitch, yaw);
	actualPose[3]=roll;
	actualPose[4]=pitch;
	actualPose[5]=yaw;
	for(int i=0; i<jointPositions.size()-1; i++)
		actualPose[6+i]=jointPositions[i];



	while(!stop_ && (finalPose-actualPose).euclideanNorm()>0.0001) {
		incrementPose=0.00001*(finalPose-actualPose);
		int index=0;
		for(int i=1; i<10; i++){
			if(abs(incrementPose[i])-maxVelocities[i]>abs(incrementPose[index])-maxVelocities[index])
				index=i;
		}
		if(abs(incrementPose[index])-maxVelocities[index]>0){
			incrementPose=(maxVelocities[index]/abs(incrementPose[index]))*incrementPose;

		}
		actualPose=actualPose+incrementPose;
		scene->iauvFile[0]->setVehiclePosition(actualPose[0],actualPose[1],actualPose[2],actualPose[3],actualPose[4],actualPose[5]);
		std::vector<double> jointPositionsNow;
		for(int i=6; i<colVector.getRows()+2; i++)
			jointPositionsNow.push_back(actualPose[i]);
		scene->iauvFile[0]->urdf->setJointPosition(jointPositionsNow);



	}

	/*int contador=0;
	double lastTime=ros::Time::now().toSec();
	while(contador<vec.size()){
		contador=0;
		double time=ros::Time::now().toSec()-lastTime;
		double move=time*0.2;
		for(int i=0; i<4; i++){
			if(vec[i]<iauvPosition[i]){
				if(iauvPosition[i]-move<=vec[i]){
					iauvPositionNow[i]=vec[i];
					contador++;
				}
				else{
					iauvPositionNow[i]=iauvPosition[i]-move;
				}
			}
			else{
				if(iauvPosition[i]+move>=vec[i]){
					iauvPositionNow[i]=vec[i];
					contador++;
				}
				else{
					iauvPositionNow[i]=iauvPosition[i]+move;
				}
			}
		}
		for(int i=4; i< vec.size(); i++){
			if(vec[i]<jointPositions[i-4]){
				if(jointPositions[i-4]-move<=vec[i]){
					jointPositionsNow[i-4]=vec[i];
					contador++;
				}
				else{
					jointPositionsNow[i-4]=jointPositions[i-4]-move;
				}
			}
			else{
				if(jointPositions[i-4]+move>=vec[i]){
					jointPositionsNow[i-4]=vec[i];
					contador++;
				}
				else{
					jointPositionsNow[i-4]=jointPositions[i-4]+move;
				}
			}
		}
		scene->iauvFile[0]->setVehiclePosition(iauvPositionNow[0],iauvPositionNow[1],iauvPositionNow[2],iauvPositionNow[3]);
		scene->iauvFile[0]->urdf->setJointPosition(jointPositionsNow);
	}
	std::cout<<"iauvPositionNow "<<iauvPositionNow[0]<<" "<<iauvPositionNow[1]<<" "<<iauvPositionNow[2]<<" "<<iauvPositionNow[3]<<std::endl;
	std::cout<<"jointPositionsNow ";
	for(int i=0; i<jointPositionsNow.size();i++){
			std::cout<<jointPositionsNow[i]<<" ";
	}
	std::cout<<std::endl;
*/
}

