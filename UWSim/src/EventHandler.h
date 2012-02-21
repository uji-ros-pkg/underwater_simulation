// ----------------------------------------------------
//                   Event Handler
// ----------------------------------------------------

class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
    osg::ref_ptr<osgOceanScene> _scene;
    osg::ref_ptr<TextHUD> _textHUD;
    osg::ref_ptr<osgWidget::Window> _window;
    

public:
    //vehicle track indicates whether the camera must automatically track the vehicle node
    SceneEventHandler( osgWidget::Window* w, TextHUD* textHUD, osg::ref_ptr<osgOceanScene> scene ):
        _scene(scene),
        _textHUD(textHUD),
	_window(w)
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

