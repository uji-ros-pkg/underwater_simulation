#ifndef MOSAIC_MANIPOLATOR_H
#define MOSAIC_MANIPULATOR_H

#include <osgGA/TrackballManipulator>
#include <osg/Vec3d>



class MosaicManipulator : public osgGA::TrackballManipulator
{
	typedef TrackballManipulator inherited;
	private:
		bool disabled;
	public:
		MosaicManipulator();
		~MosaicManipulator();
		bool trackball(osg::Vec3d&, float&, float, float, float, float);
		void rotateTrackball( const float, const float ,const float, const float, const float);
		void panModel( const float, const float, const float );
		void zoomModel( const float, bool );
		void disableManipulator(bool);

		
	
};







#endif
