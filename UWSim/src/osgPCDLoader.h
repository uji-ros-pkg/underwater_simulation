#ifndef OSGPCDLOADER_H_
#define OSGPCDLOADER_H_

#include "SimulatorConfig.h"

#ifdef BUILD_ROS_INTERFACES

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <osg/Geometry>
#include <osg/Geode>

class osgPCDLoader {
	public:

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	osg::ref_ptr<osg::Geode> geode;

	osgPCDLoader(std::string pcd_file) {
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file, cloud) == -1) //* load the file
		{
			std::cerr << "Couldn't read file " << pcd_file << std::endl;
		} else {
			std::cout << "Loaded "
			    	<< cloud.width * cloud.height
			   	<< " data points from " << pcd_file
			    	<< std::endl;	

			geode=osg::ref_ptr<osg::Geode>(new osg::Geode());
			osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());
			osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
			osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());

			//Read vertex and color info from PCD
			for (int i=0; i<cloud.points.size(); i++) {
				vertices->push_back (osg::Vec3 (cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));

				uint32_t rgb_val_;
				memcpy(&rgb_val_, &(cloud.points[i].rgb), sizeof(uint32_t));
		
				uint32_t red,green,blue;
				blue=rgb_val_ & 0x000000ff;
				rgb_val_ = rgb_val_ >> 8;
				green=rgb_val_ & 0x000000ff;
				rgb_val_ = rgb_val_ >> 8;
				red=rgb_val_ & 0x000000ff;

				colors->push_back (osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f));
			}
	
			//Set OSG Geometry vertex and colors
			geometry->setVertexArray(vertices.get());
			geometry->setColorArray(colors.get());
			geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

			geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

			geode->addDrawable (geometry.get());
			osg::StateSet* state = geometry->getOrCreateStateSet();
			state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
		}
	}
	
	~osgPCDLoader(){}

};

#endif
#endif

