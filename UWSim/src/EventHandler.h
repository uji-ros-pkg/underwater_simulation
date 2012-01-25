// ----------------------------------------------------
//                   Event Handler
// ----------------------------------------------------

class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
    osg::ref_ptr<osgOceanScene> _scene;
    osg::ref_ptr<TextHUD> _textHUD;
    osgWidget::Window* w;
    

public:
    //vehicle track indicates whether the camera must automatically track the vehicle node
    SceneEventHandler( osgWidget::Window* w, TextHUD* textHUD, osg::ref_ptr<osgOceanScene> scene ):
        _scene(scene),
        _textHUD(textHUD)
    {
		this->w=w;	
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
		    if (w->isVisible()) w->hide(); else w->show();
                    return false;
                }
		
	   }  
        default:
            return false;
        }
    }

    void getUsage(osg::ApplicationUsage& usage) const
    {
        //usage.addKeyboardMouseBinding("c","Camera type (cycle through Fixed, Flight, Trackball)");
        //usage.addKeyboardMouseBinding("1","Select scene \"Clear Blue Sky\"");
        //usage.addKeyboardMouseBinding("2","Select scene \"Dusk\"");
        //usage.addKeyboardMouseBinding("3","Select scene \"Pacific Cloudy\"");
    }

};

