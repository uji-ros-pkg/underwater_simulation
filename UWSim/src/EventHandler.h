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

#ifndef SCENE_EVENT_HANDLER
#define SCENE_EVENT_HANDLER

#include "SimulatedIAUV.h"
#include "ROSInterface.h"
#include "ConfigXMLParser.h"

class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
    osg::ref_ptr<osgOceanScene> _scene;
    osg::ref_ptr<TextHUD> _textHUD;
    std::vector<osg::ref_ptr<osgWidget::Window> > _windows;
    std::vector<boost::shared_ptr<ROSInterface> > _ROSInterfaces;
    ConfigFile *_config;

    bool draw_frames_;    
public:
    //vehicle track indicates whether the camera must automatically track the vehicle node
    SceneEventHandler( std::vector<osg::ref_ptr<osgWidget::Window> > &windows, TextHUD* textHUD, osg::ref_ptr<osgOceanScene> scene, std::vector<boost::shared_ptr<ROSInterface> > &ROSInterfaces, ConfigFile *config):
        _scene(scene),
        _textHUD(textHUD),
	_windows(windows),
	_ROSInterfaces(ROSInterfaces),
	draw_frames_(false)
    {
        	_textHUD->setSceneText("Clear Blue Sky");
		_config=config;
    }

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::KEYUP):
            {
		
                if(ea.getKey() == '1')
                {
		    ROS_INFO("Switch to Clear Blue Sky");
                    _scene->changeScene( osgOceanScene::CLEAR );
        	    _scene->getOceanScene()->setUnderwaterFog(_config->fogDensity,  osg::Vec4f(_config->fogColor[0],_config->fogColor[1],_config->fogColor[2],1) );
        	    _scene->getOceanScene()->setUnderwaterDiffuse( osg::Vec4f(_config->color[0],_config->color[1],_config->color[2],1) );
        	    _scene->getOceanScene()->setUnderwaterAttenuation( osg::Vec3f(_config->attenuation[0], _config->attenuation[1], _config->attenuation[2]) );

                    _textHUD->setSceneText( "Clear Blue Sky" );
                    return false;
                }
                else if(ea.getKey() == '2')
                {
                    _scene->changeScene( osgOceanScene::DUSK );
                    _textHUD->setSceneText( "Dusk" );
        	    _scene->getOceanScene()->setUnderwaterFog(_config->fogDensity,  osg::Vec4f(_config->fogColor[0],_config->fogColor[1],_config->fogColor[2],1) );
        	    _scene->getOceanScene()->setUnderwaterDiffuse( osg::Vec4f(_config->color[0],_config->color[1],_config->color[2],1) );
        	    _scene->getOceanScene()->setUnderwaterAttenuation( osg::Vec3f(_config->attenuation[0], _config->attenuation[1], _config->attenuation[2]) );
                    return false;
                }
                else if(ea.getKey() == '3' )
		{
                    _scene->changeScene( osgOceanScene::CLOUDY );
                    _textHUD->setSceneText( "Pacific Cloudy" );
        	    _scene->getOceanScene()->setUnderwaterFog(_config->fogDensity,  osg::Vec4f(_config->fogColor[0],_config->fogColor[1],_config->fogColor[2],1) );
        	    _scene->getOceanScene()->setUnderwaterDiffuse( osg::Vec4f(_config->color[0],_config->color[1],_config->color[2],1) );
        	    _scene->getOceanScene()->setUnderwaterAttenuation( osg::Vec3f(_config->attenuation[0], _config->attenuation[1], _config->attenuation[2]) );
                    return false;
                } else if(ea.getKey() == 'c' )
		{
		    for (unsigned int i=0; i<_windows.size(); i++)
		    	if (_windows[i]->isVisible()) _windows[i]->hide(); else _windows[i]->show();
                    return false;
                } else if (ea.getKey() == 'f' ) {
			//Search for 'switch_frames' nodes and toggle their values
			findNodeVisitor finder("switch_frames");
			_scene->localizedWorld->accept(finder);
			std::vector<osg::Node*> node_list=finder.getNodeList();
			(draw_frames_) ? draw_frames_=false : draw_frames_=true;
			for (unsigned int i=0; i<node_list.size(); i++) 
			  (draw_frames_) ? node_list[i]->asSwitch()->setAllChildrenOn() : node_list[i]->asSwitch()->setAllChildrenOff();
		} else if (ea.getKey() == 'r' ) {
  		  //search catchable objects and get them back to their original positions
		  GetCatchableObjects finder;
		  _scene->getScene()->accept(finder);
		  std::vector<osg::Node*> node_list=finder.getNodeList();
		  for (unsigned int i=0; i<node_list.size(); i++) {
		    osg::ref_ptr<NodeDataType> data = dynamic_cast<NodeDataType*> (node_list[i]->getUserData());
		    osg::Matrixd matrix;
	            matrix.makeRotate(osg::Quat(data->originalRotation[0],osg::Vec3d(1,0,0),data->originalRotation[1],osg::Vec3d(0,1,0), data->originalRotation[2],osg::Vec3d(0,0,1) ));
		    matrix.setTrans(data->originalPosition[0],data->originalPosition[1],data->originalPosition[2]);
		    node_list[i]->asTransform()->asMatrixTransform()->setMatrix(matrix);
		    _scene->localizedWorld->addChild(node_list[i]);
		    node_list[i]->getParent(0)->removeChild(node_list[i]);
		  }

		  //Search for object picker to change picked to false
		  findNodeVisitor finder2("ObjectPickerNode");
		  _scene->localizedWorld->accept(finder2);
		  node_list=finder2.getNodeList();

 		  for (unsigned int i=0; i<node_list.size(); i++) {
		    osg::ref_ptr<ObjectPickerUpdateCallback> callback = dynamic_cast<ObjectPickerUpdateCallback*> (node_list[i]->getUpdateCallback());
		    callback->picked=false;
		  }

		  //Search for ROSOdomToPAT to restart waypoints
		  for (unsigned int i=0; i<_ROSInterfaces.size();i++){
		    ROSOdomToPAT * iface=  dynamic_cast<ROSOdomToPAT *> (_ROSInterfaces[i].get());
		    if(iface)
		      iface->clearWaypoints();
 		  }
		}
		
		
		
	   }  
        default:
            return false;
        }
    }

    void getUsage(osg::ApplicationUsage& usage) const
    {
    }

};

#endif
