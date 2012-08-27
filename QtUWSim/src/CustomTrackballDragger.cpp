/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
 */
//osgManipulator - Copyright (C) 2007 Fugro-Jason B.V.
//Customized for QtUWSim

#include "CustomTrackballDragger.h"
#include <osgManipulator/AntiSquish>

#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/PolygonMode>
#include <osg/CullFace>
#include <osg/Quat>
#include <osg/AutoTransform>
#include <osg/Material>

#include <math.h>

using namespace osgManipulator;

namespace
{

osg::Geometry* createDisk(float ext_radius, float int_radius, unsigned int numSegments, float opacity=1.0)
{
	const float angleDelta = 4.0f*osg::PI/(float)numSegments;
	const float er = ext_radius;
	const float ir = int_radius;
	float angle = 0.0f;
	osg::Vec3Array* vertexArray = new osg::Vec3Array(numSegments*2+2);
	osg::Vec3Array* normalArray = new osg::Vec3Array(1);
	(*normalArray)[0].set(0.0f,0.0f,1.0f);
	for(unsigned int i = 0; i <= numSegments; i+=2,angle+=angleDelta)
	{
		float c = cosf(angle);
		float s = sinf(angle);
		(*vertexArray)[i].set(c*ir,s*ir,0.0f);
		(*vertexArray)[i+1].set(c*er,s*er,0.0f);
	}

	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray(vertexArray);
	geometry->setNormalArray(normalArray);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP,0,vertexArray->size()));
	geometry->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);


	//Disk transparency
	osg::StateSet* state = geometry->getOrCreateStateSet();
	state->setMode(GL_BLEND,osg::StateAttribute::ON|
			osg::StateAttribute::OVERRIDE);
	osg::Material* mat = (osg::Material*)state->getAttribute
			(osg::StateAttribute::MATERIAL);
	if(!mat) {
		mat = new osg::Material;
	}
	mat->setAlpha(osg::Material::FRONT_AND_BACK, opacity);
	state->setAttributeAndModes(mat,osg::StateAttribute::ON);
	geometry->setStateSet(state);

	return geometry;
}


}

CustomTrackballDragger::CustomTrackballDragger(bool useAutoTransform)
{
	if (useAutoTransform)
	{
		float pixelSize = 40.0f;
		osg::MatrixTransform* scaler = new osg::MatrixTransform;
		scaler->setMatrix(osg::Matrix::scale(pixelSize, pixelSize, pixelSize));

		osg::AutoTransform *at = new osg::AutoTransform;
		at->setAutoScaleToScreen(true);
		at->addChild(scaler);

		AntiSquish* as = new AntiSquish;
		as->addChild(at);
		addChild(as);

		_yDragger = new RotateCylinderDragger();
		scaler->addChild(_yDragger.get());
		addDragger(_yDragger.get());
	}
	else
	{
		AntiSquish* as = new AntiSquish;
		//as->addChild(at);
		addChild(as);

		_yDragger = new RotateCylinderDragger();
		as->addChild(_yDragger.get());
		addDragger(_yDragger.get());
	}

	setParentDragger(getParentDragger());
}

CustomTrackballDragger::~CustomTrackballDragger()
{
}

void CustomTrackballDragger::setupDefaultGeometry()
{
	osg::Geode* geode = new osg::Geode;

	geode->setName("Dragger Handle");
	geode->addDrawable(createDisk(0.4f, 0.3f, 80, 0.5));

	geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	_yDragger->addChild(geode);

	// Rotate Y-axis dragger appropriately.
	{
		osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
		osg::Matrix m;
		m.setRotate(osg::Quat(M_PI_2, osg::Vec3d(1,0,0)));
		m.setTrans(0,0,0);
		_yDragger->setMatrix(m);
	}

	_yDragger->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
}
