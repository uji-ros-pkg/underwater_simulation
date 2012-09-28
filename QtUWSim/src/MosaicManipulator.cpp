#include "MosaicManipulator.h"
#include  <osgGA/GUIEventAdapter>
#include <osg/Vec3d>
#include <iostream>


using namespace osgGA;
using namespace osg;
using namespace std;

MosaicManipulator::MosaicManipulator():TrackballManipulator()
{
	disabled=false;
	}
MosaicManipulator::~MosaicManipulator(){
}
bool MosaicManipulator::trackball(osg::Vec3d& axis, float& angle, float p1x, float p1y, float p2x, float p2y){

	float a=sqrt(pow(p1x,2)+pow(p1y,2));
	float b=sqrt(pow(p2x,2)+pow(p2y,2));
	if(a==0 || b==0){
		angle=0;
	}
	else
	{
		angle=osg::inRadians(acos((p1x*p2x+p1y*p2y)/(a*b)));
		
		
	}
	if((p1x*p2y)-(p1y*p2x)>0)
		axis=osg::Vec3d(0.0f,0.0f,-1.0f);
	else
		axis=osg::Vec3d(0.0f,0.0f,1.0f);   
	return true; 
}
void MosaicManipulator::rotateTrackball( const float px0, const float py0,
                                        const float px1, const float py1, const float scale )
{
	if(!disabled){
    osg::Vec3d axis;
    float angle;

    trackball( axis, angle, px1, py1, px0, py0 );
    Quat new_rotate;
    new_rotate.makeRotate( angle, axis );

    _rotation = _rotation * new_rotate;
}
}

void MosaicManipulator::panModel( const float dx, const float dy, const float dz )
{
	if(!disabled){
    Matrix rotation_matrix;
    rotation_matrix.makeRotate( _rotation );

    Vec3d dv( dx, dy, dz );

    _center += dv * rotation_matrix;
}
}


void MosaicManipulator::zoomModel( const float dy, bool pushForwardIfNeeded )
{
	if(!disabled){
    // scale
    float scale = 1.0f + dy;

    // minimum distance
    float minDist = _minimumDistance;
    if( getRelativeFlag( _minimumDistanceFlagIndex ) )
        minDist *= _modelSize;

    if( _distance*scale > minDist )
    {
        // regular zoom
        _distance *= scale;
    }
    else
    {
        if( pushForwardIfNeeded )
        {
            // push the camera forward
            float scale = -_distance;
            Matrixd rotation_matrix( _rotation );
            Vec3d dv = (Vec3d( 0.0f, 0.0f, -1.0f ) * rotation_matrix) * (dy * scale);
            _center += dv;
        }
        else
        {
            // set distance on its minimum value
            _distance = minDist;
        }
    }
	}	
}




void MosaicManipulator::disableManipulator(bool disable){
	disabled=disable;
}

