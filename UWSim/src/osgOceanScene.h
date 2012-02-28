#ifndef OSGOCEANSCENE_H_
#define OSGOCEANSCENE_H_

#include "SimulatorConfig.h"
#include "UWSimUtils.h"

#include <osg/TextureCubeMap>
#include <osgDB/ReadFile>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osgText/Text>
#include <osg/CullFace>
#include <osg/Fog>
#include <osgText/Font>
#include <osg/Switch>
#include <osg/Texture3D>
#include <osg/LightSource>
#include <osg/Image>

#include <string>
#include <string.h>
#include <vector>

#include <osgOcean/Version>
#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/SiltEffect>
#include <osgOcean/ShaderManager>

#include "SkyDome.h"
#include "ConfigXMLParser.h"
#include "UWSimUtils.h"

// ----------------------------------------------------
//               Camera Track Callback
// ----------------------------------------------------

class CameraTrackCallback: public osg::NodeCallback
{
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            osg::Vec3f centre,up,eye;
            // get MAIN camera eye,centre,up
            cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye,centre,up);
            // update position
            osg::MatrixTransform* mt = static_cast<osg::MatrixTransform*>(node);
            mt->setMatrix( osg::Matrix::translate( eye.x(), eye.y(), mt->getMatrix().getTrans().z() ) );
        }

        traverse(node, nv);
    }
};

class osgOceanScene : public osg::Referenced
{
public:
    enum SCENE_TYPE{ CLEAR, DUSK, CLOUDY };
    osg::ref_ptr<osg::MatrixTransform> localizedWorld;

private:
    SCENE_TYPE _sceneType;

    osg::ref_ptr<osgText::Text> _modeText;
    osg::ref_ptr<osg::Group> _scene;

    osg::ref_ptr<osgOcean::OceanScene> _oceanScene;
    osg::ref_ptr<osgOcean::FFTOceanSurface> _oceanSurface;
    osg::ref_ptr<osg::TextureCubeMap> _cubemap;
    osg::ref_ptr<SkyDome> _skyDome;

    std::vector<std::string> _cubemapDirs;
    std::vector<osg::Vec4f>  _lightColors;
    std::vector<osg::Vec4f>  _fogColors;
    std::vector<osg::Vec3f>  _underwaterAttenuations;
    std::vector<osg::Vec4f>  _underwaterDiffuse;

    osg::ref_ptr<osg::Light> _light;

    std::vector<osg::Vec3f>  _sunPositions;
    std::vector<osg::Vec4f>  _sunDiffuse;
    std::vector<osg::Vec4f>  _waterFogColors;

    osg::ref_ptr<osg::Switch> _islandSwitch;

public:

    osgOceanScene(double offsetp[3], double offsetr[3],const osg::Vec2f& windDirection = osg::Vec2f(1.0f,1.0f),
                float windSpeed = 12.f,
                float depth = 10000.f,
                float reflectionDamping = 0.35f,
                float scale = 1e-8,
                bool  isChoppy = true,
                float choppyFactor = -2.5f,
                float crestFoamHeight = 2.2f):
        _sceneType(DUSK)
    {
        _cubemapDirs.push_back( std::string("sky_clear") );
        _cubemapDirs.push_back( std::string("sky_dusk") );
        _cubemapDirs.push_back( std::string("sky_fair_cloudy") );

        _fogColors.push_back( intColor( 199,226,255 ) );
        _fogColors.push_back( intColor( 244,228,179 ) );
        _fogColors.push_back( intColor( 172,224,251 ) );

        _waterFogColors.push_back( intColor(0,0,100) );
        //_waterFogColors.push_back( intColor(27,57,109) );
        _waterFogColors.push_back( intColor(44,69,106 ) );
        _waterFogColors.push_back( intColor(84,135,172 ) );

        _underwaterAttenuations.push_back( osg::Vec3f(0.015f, 0.0075f, 0.005f) );
        _underwaterAttenuations.push_back( osg::Vec3f(0.015f, 0.0075f, 0.005f) );
        _underwaterAttenuations.push_back( osg::Vec3f(0.008f, 0.003f, 0.002f) );

        _underwaterDiffuse.push_back( intColor(0,0,100) );
        //_underwaterDiffuse.push_back( intColor(27,57,109) );
        _underwaterDiffuse.push_back( intColor(44,69,106) );
        _underwaterDiffuse.push_back( intColor(84,135,172) );

        _lightColors.push_back( intColor( 105,138,174 ) );
        _lightColors.push_back( intColor( 105,138,174 ) );
        _lightColors.push_back( intColor( 105,138,174 ) );

        _sunPositions.push_back( osg::Vec3f(326.573, 1212.99 ,1275.19) );
        _sunPositions.push_back( osg::Vec3f(520.f, 1900.f, 550.f) );
        _sunPositions.push_back( osg::Vec3f(-1056.89f, -771.886f, 1221.18f ) );

        _sunDiffuse.push_back( intColor( 191, 191, 191 ) );
        _sunDiffuse.push_back( intColor( 251, 251, 161 ) );
        _sunDiffuse.push_back( intColor( 191, 191, 191 ) );

        build(offsetp, offsetr, windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, crestFoamHeight);
    }

    void build( double offsetp[3],
		double offsetr[3],
		const osg::Vec2f& windDirection,
                float windSpeed,
                float depth,
                float reflectionDamping,
                float waveScale,
                bool  isChoppy,
                float choppyFactor,
                float crestFoamHeight)
    {
        {
            ScopedTimer buildSceneTimer("Building scene... \n", osg::notify(osg::ALWAYS));

            _scene = new osg::Group;

            {
                ScopedTimer cubemapTimer("  . Loading cubemaps: ", osg::notify(osg::ALWAYS));
                _cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );
            }

            // Set up surface
            {
                ScopedTimer oceanSurfaceTimer("  . Generating ocean surface: ", osg::notify(osg::ALWAYS));
                _oceanSurface = new osgOcean::FFTOceanSurface( 64, 256, 17,
                    windDirection, windSpeed, depth, reflectionDamping, waveScale, isChoppy, choppyFactor, 10.f, 256 );

                _oceanSurface->setEnvironmentMap( _cubemap.get() );
                _oceanSurface->setFoamBottomHeight( 2.2f );
                _oceanSurface->setFoamTopHeight( 3.0f );
                _oceanSurface->enableCrestFoam( true );
                _oceanSurface->setLightColor( _lightColors[_sceneType] );
                // Make the ocean surface track with the main camera position, giving the illusion
                // of an endless ocean surface.
                _oceanSurface->enableEndlessOcean(true);
            }

            // Set up ocean scene, add surface
            {
                ScopedTimer oceanSceneTimer("  . Creating ocean scene: ", osg::notify(osg::ALWAYS));
                osg::Vec3f sunDir = -_sunPositions[_sceneType];
                sunDir.normalize();

                _oceanScene = new osgOcean::OceanScene( _oceanSurface.get() );
                _oceanScene->setLightID(0);
                _oceanScene->enableReflections(true);
                _oceanScene->enableRefractions(true);

                // Set the size of _oceanCylinder which follows the camera underwater.
                // This cylinder prevents the clear from being visible past the far plane
                // instead it will be the fog color.
                // The size of the cylinder should be changed according the size of the ocean surface.
                _oceanScene->setCylinderSize( 1500.f, 4000.f );

                _oceanScene->setAboveWaterFog(0.0012f, _fogColors[_sceneType] );
                _oceanScene->setUnderwaterFog(0.060f,  _waterFogColors[_sceneType] );
                _oceanScene->setUnderwaterDiffuse( _underwaterDiffuse[_sceneType] );
                _oceanScene->setUnderwaterAttenuation( _underwaterAttenuations[_sceneType] );
 //               _oceanScene->setDOFNear(0.f);
 //               _oceanScene->setDOFFar(0.f);

                _oceanScene->setSunDirection( sunDir );
                _oceanScene->enableGodRays(true);
                _oceanScene->enableSilt(true);
                _oceanScene->enableUnderwaterDOF(false);
                _oceanScene->enableDistortion(true);
                _oceanScene->enableGlare(false);
                _oceanScene->setGlareAttenuation(0.8f);


                // create sky dome and add to ocean scene
                // set masks so it appears in reflected scene and normal scene
                _skyDome = new SkyDome( 1900.f, 16, 16, _cubemap.get() );
                _skyDome->setNodeMask( _oceanScene->getReflectedSceneMask() | _oceanScene->getNormalSceneMask() );

                // add a pat to track the camera
                osg::MatrixTransform* transform = new osg::MatrixTransform;
                transform->setDataVariance( osg::Object::DYNAMIC );
                transform->setMatrix( osg::Matrixf::translate( osg::Vec3f(0.f, 0.f, 0.f) ));
                transform->setCullCallback( new CameraTrackCallback );

                transform->addChild( _skyDome.get() );
		transform->setName("camara-cielo");
                _oceanScene->addChild( transform );

                {
                    // Create and add fake texture for use with nodes without any texture
                    // since the OceanScene default scene shader assumes that texture unit
                    // 0 is used as a base texture map.
                    osg::Image * image = new osg::Image;
                    image->allocateImage( 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE );
                    *(osg::Vec4ub*)image->data() = osg::Vec4ub( 0xFF, 0xFF, 0xFF, 0xFF );

                    osg::Texture2D* fakeTex = new osg::Texture2D( image );
                    fakeTex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
                    fakeTex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
                    fakeTex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
                    fakeTex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);

                    osg::StateSet* stateset = _oceanScene->getOrCreateStateSet();
                    stateset->setTextureAttribute(0,fakeTex,osg::StateAttribute::ON);
                    stateset->setTextureMode(0,GL_TEXTURE_1D,osg::StateAttribute::OFF);
                    stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
                    stateset->setTextureMode(0,GL_TEXTURE_3D,osg::StateAttribute::OFF);
                }
            }

            {
                ScopedTimer lightingTimer("  . Setting up lighting: ", osg::notify(osg::ALWAYS));
                osg::LightSource* lightSource = new osg::LightSource;
                lightSource->setLocalStateSetModes();

                _light = lightSource->getLight();
                _light->setLightNum(0);
                _light->setAmbient( osg::Vec4d(0.3f, 0.3f, 0.3f, 1.0f ));
                _light->setDiffuse( _sunDiffuse[_sceneType] );
                _light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
                _light->setPosition( osg::Vec4f(_sunPositions[_sceneType], 1.f) ); // point light

                _scene->addChild( lightSource );
                _scene->addChild( _oceanScene.get() );
            } 
	    {
		//Add a coordinate transform relating the simulated world frame with respect to an arbitrary localized world
                ScopedTimer lightingTimer("  . Setting localized world: ", osg::notify(osg::ALWAYS));
		osg::Matrixd wMl;
		wMl.makeRotate(offsetr[0],1,0,0);
		wMl.preMultRotate(osg::Quat(offsetr[1],osg::Vec3d(0,1,0)));
		wMl.preMultRotate(osg::Quat(offsetr[2],osg::Vec3d(0,0,1)));
		wMl.setTrans(offsetp[0],offsetp[1],offsetp[2]);
		localizedWorld=new osg::MatrixTransform(wMl);
		//add frame to localized world
		osg::ref_ptr<osg::Node> axis=UWSimGeometry::createSwitchableFrame();
		localizedWorld->asGroup()->addChild(axis);
		
		_oceanScene->addChild(localizedWorld);
	    }

            osg::notify(osg::ALWAYS) << "Complete. Time Taken: ";
        }
    }

    osgOcean::OceanTechnique* getOceanSurface( void )
    {
        return _oceanSurface.get();
    }

    osg::Group* getScene(void){
        return _scene.get();
    }

    osgOcean::OceanScene* getOceanScene()
    {
        return _oceanScene.get();
    }

    void changeScene( SCENE_TYPE type )
    {
	OSG_DEBUG << "scene type" << std::endl;
        SCENE_TYPE _sceneType = type;

	OSG_DEBUG << "scenetype: " << _sceneType << std::endl;

        //_cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );
	OSG_DEBUG << "loag Cubemaps" << std::endl;
        _cubemap = loadCubeMapTextures("sky_dusk" );
	OSG_DEBUG << "set Cubemaps" << std::endl;
        _skyDome->setCubeMap( _cubemap.get() );
	OSG_DEBUG << " set Env Map" << std::endl;
        _oceanSurface->setEnvironmentMap( _cubemap.get() );
	OSG_DEBUG << "set light color" << std::endl;
        _oceanSurface->setLightColor( _lightColors[type] );

	OSG_DEBUG << "fog" << std::endl;
        _oceanScene->setAboveWaterFog(0.0012f, _fogColors[_sceneType] );
        _oceanScene->setUnderwaterFog(0.06f,  _waterFogColors[_sceneType] );
        _oceanScene->setUnderwaterDiffuse( _underwaterDiffuse[_sceneType] );
        _oceanScene->setUnderwaterAttenuation( _underwaterAttenuations[_sceneType] );

	OSG_DEBUG << "sundir" << std::endl;
        osg::Vec3f sunDir = -_sunPositions[_sceneType];
        sunDir.normalize();

        _oceanScene->setSunDirection( sunDir );

	OSG_DEBUG << "sunpos" << std::endl;
        _light->setPosition( osg::Vec4f(_sunPositions[_sceneType],1.f) );
        _light->setDiffuse( _sunDiffuse[_sceneType] ) ;
	OSG_DEBUG << "done" << std::endl;
        //if(_islandSwitch.valid() )
        //{
        //    if(_sceneType == CLEAR || _sceneType == CLOUDY)
        //        _islandSwitch->setAllChildrenOn();
        //    else
        //        _islandSwitch->setAllChildrenOff();
        //}
    }

    osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures( const std::string& dir )
    {
	OSG_DEBUG << "start cubemaps" << std::endl;
        enum {POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z};

	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/textures/")+dir);

        std::string filenames[6];
	OSG_DEBUG << "set filenames" << std::endl;
        filenames[POS_X] = std::string("east.png");
        filenames[NEG_X] = std::string("west.png");
        filenames[POS_Z] = std::string("north.png");
        filenames[NEG_Z] = std::string("south.png");
        filenames[POS_Y] = std::string("down.png");
        filenames[NEG_Y] = std::string("up.png");

	OSG_DEBUG << "set textureCubeMap" << std::endl;
        osg::ref_ptr<osg::TextureCubeMap> cubeMap = new osg::TextureCubeMap;
        cubeMap->setInternalFormat(GL_RGBA);

	OSG_DEBUG << "set filters" << std::endl;
        cubeMap->setFilter( osg::Texture::MIN_FILTER,    osg::Texture::LINEAR_MIPMAP_LINEAR);
        cubeMap->setFilter( osg::Texture::MAG_FILTER,    osg::Texture::LINEAR);
        cubeMap->setWrap  ( osg::Texture::WRAP_S,        osg::Texture::CLAMP_TO_EDGE);
        cubeMap->setWrap  ( osg::Texture::WRAP_T,        osg::Texture::CLAMP_TO_EDGE);

	OSG_DEBUG << "set image" << std::endl;
        cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, osgDB::readImageFile( filenames[NEG_X] ) );
        cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, osgDB::readImageFile( filenames[POS_X] ) );
        cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, osgDB::readImageFile( filenames[NEG_Y] ) );
        cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, osgDB::readImageFile( filenames[POS_Y] ) );
        cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, osgDB::readImageFile( filenames[NEG_Z] ) );
        cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, osgDB::readImageFile( filenames[POS_Z] ) );

	OSG_DEBUG << "done" << std::endl;
        return cubeMap;
    }

    osg::Geode* sunDebug( const osg::Vec3f& position )
    {
        osg::ShapeDrawable* sphereDraw = new osg::ShapeDrawable( new osg::Sphere( position, 15.f ) );
        sphereDraw->setColor(osg::Vec4f(1.f,0.f,0.f,1.f));

        osg::Geode* sphereGeode = new osg::Geode;
        sphereGeode->addDrawable( sphereDraw );

        return sphereGeode;
    }

    osg::Vec4f intColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a = 255 )
    {
        float div = 1.f/255.f;
        return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
    }

    osgOcean::OceanScene::EventHandler* getOceanSceneEventHandler()
    {
        return _oceanScene->getEventHandler();
    }

    osg::Node* addObject(osg::Transform *transform, std::string filename, Object *o=NULL)
    {
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH));
        osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/objects"));
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/terrain"));
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/shaders"));
        osg::ref_ptr<osg::Node> object = osgDB::readNodeFile(filename);

        if(!object.valid()){
            osg::notify(osg::WARN) << "Could not find: " << filename << std::endl;
            return NULL;
        } else {
			static const char model_vertex[]   = "default_scene.vert";
			static const char model_fragment[] = "default_scene.frag";

			osg::Program* program = osgOcean::ShaderManager::instance().createProgram("object_shader", model_vertex, model_fragment, true);
			program->addBindAttribLocation("aTangent", 6);

			object->getOrCreateStateSet()->setAttributeAndModes(program,osg::StateAttribute::ON);
			object->getStateSet()->addUniform( new osg::Uniform( "uOverlayMap", 1 ) );
			object->getStateSet()->addUniform( new osg::Uniform( "uNormalMap",  2 ) );
			object->setNodeMask( _oceanScene->getNormalSceneMask() | _oceanScene->getReflectedSceneMask() | _oceanScene->getRefractedSceneMask() );

			osg::Matrix linkBase;
			linkBase.makeIdentity();
			linkBase.preMultRotate(osg::Quat(o->offsetr[0],osg::Vec3d(1,0,0)));
			linkBase.preMultRotate(osg::Quat(o->offsetr[1],osg::Vec3d(0,1,0)));
			linkBase.preMultRotate(osg::Quat(o->offsetr[2],osg::Vec3d(0,0,1)));
			linkBase.preMultTranslate(osg::Vec3d(-o->offsetp[0],-o->offsetp[1],-o->offsetp[2]));
   
			osg::ref_ptr<osg::MatrixTransform> linkBaseTransform= new osg::MatrixTransform(linkBase);
			linkBaseTransform->addChild(object);

			osg::Matrix linkPost;
			linkBase.invert(linkPost);

			osg::ref_ptr<osg::MatrixTransform> linkPostTransform= new osg::MatrixTransform(linkPost);
			object->asGroup()->addChild(linkPostTransform);

			transform->addChild(linkBaseTransform);
			localizedWorld->addChild(transform);
			return object.get();
        }
    }

    void addObject(osg::Transform *transform)
    {
  	if (transform!=NULL)
		localizedWorld->addChild (transform ); 
    }
};

#endif /* OSGOCEANSCENE_H_ */
