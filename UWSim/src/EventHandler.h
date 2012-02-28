#ifndef SCENE_EVENT_HANDLER
#define SCENE_EVENT_HANDLER

#include "SimulatedIAUV.h"


class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
    osg::ref_ptr<osgOceanScene> _scene;
    osg::ref_ptr<TextHUD> _textHUD;
    osg::ref_ptr<osgWidget::Window> _window;

    bool draw_frames_;    
public:
    //vehicle track indicates whether the camera must automatically track the vehicle node
    SceneEventHandler( osgWidget::Window* w, TextHUD* textHUD, osg::ref_ptr<osgOceanScene> scene):
        _scene(scene),
        _textHUD(textHUD),
	_window(w),
	draw_frames_(false)
    {
        	_textHUD->setSceneText("Clear Blue Sky");
    }

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::KEYUP):
            {
		
                if(ea.getKey() == '1')
                {
                    _scene->changeScene( osgOceanScene::CLEAR );
                    _textHUD->setSceneText( "Clear Blue Sky" );
                    return false;
                }
                else if(ea.getKey() == '2')
                {
                    _scene->changeScene( osgOceanScene::DUSK );
                    _textHUD->setSceneText( "Dusk" );
                    return false;
                }
                else if(ea.getKey() == '3' )
		{
                    _scene->changeScene( osgOceanScene::CLOUDY );
                    _textHUD->setSceneText( "Pacific Cloudy" );
                    return false;
                } else if(ea.getKey() == 'c' )
		{
		    if (_window->isVisible()) _window->hide(); else _window->show();
                    return false;
                } else if (ea.getKey() == 'f' ) {
			//Search for 'switch_frames' nodes and toggle their values
			findNodeVisitor finder("switch_frames");
			_scene->localizedWorld->accept(finder);
			std::vector<osg::Node*> node_list=finder.getNodeList();
			(draw_frames_) ? draw_frames_=false : draw_frames_=true;
			for (unsigned int i=0; i<node_list.size(); i++) 
			  (draw_frames_) ? node_list[i]->asSwitch()->setAllChildrenOn() : node_list[i]->asSwitch()->setAllChildrenOff();
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
