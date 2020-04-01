#include "GLViewFrustumCulling.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "WOGUILabel.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "AftrGLRendererBase.h"

//If we want to use way points, we need to include this.
#include "FrustumCullingWayPoints.h"

#include <cmath>
#include "MGLFrustum.h"

using namespace Aftr;

GLViewFrustumCulling* GLViewFrustumCulling::New( const std::vector< std::string >& args )
{
   GLViewFrustumCulling* glv = new GLViewFrustumCulling( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewFrustumCulling::GLViewFrustumCulling( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewFrustumCulling::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewFrustumCulling::onCreate() is invoked after this module's LoadMap() is completed.
	doRotate = true;
}


void GLViewFrustumCulling::onCreate()
{
   //GLViewFrustumCulling::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}


GLViewFrustumCulling::~GLViewFrustumCulling()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewFrustumCulling::updateWorld()
{
	GLView::updateWorld(); //Just call the parent's update world first.
						   //If you want to add additional functionality, do it after
						   //this call.
	Vector camLookDirection = cam->getLookDirection();

	cameraFrustum->setPosition(cam->getPosition());
	cameraFrustum->getModel()->setLookDirection(camLookDirection);

	if (doRotate)
	nonCameraFrustum->getModel()->setLookDirection(Vector(camLookDirection.x, camLookDirection.y, 0));

	doFrustumCulling();
}


void GLViewFrustumCulling::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewFrustumCulling::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewFrustumCulling::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewFrustumCulling::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewFrustumCulling::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   // Swap the current frustum
   if( key.keysym.sym == SDLK_1 )
   {
	   if (currentFrustum == cameraFrustum) {
		   currentFrustum = nonCameraFrustum;
		   nonCameraFrustum->isVisible = true;
	   }
	   else {
		   currentFrustum = cameraFrustum;
		   nonCameraFrustum->isVisible = false;
	   }
   }
   // Lock/Unlock the nonCameraFrustum
   if (key.keysym.sym == SDLK_2)
   {
	   doRotate = !doRotate;
   }
}


void GLViewFrustumCulling::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void Aftr::GLViewFrustumCulling::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );
   this->cam->setLabel("Camera");

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );

   std::string wheeledCar(ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl");
   std::string stone(ManagerEnvironmentConfiguration::getSMM() + "/models/stone.wrl");
   std::string spaceAsteroid(ManagerEnvironmentConfiguration::getSMM() + "/models/spaceAsteroid.wrl");
   std::string spaceAsteroid2(ManagerEnvironmentConfiguration::getSMM() + "/models/spaceAsteroid2.wrl");
   std::string spaceSatellite(ManagerEnvironmentConfiguration::getSMM() + "/models/spaceSatellite.wrl");
   std::string techbox1(ManagerEnvironmentConfiguration::getSMM() + "/models/techbox1.wrl");
   std::string techbox2(ManagerEnvironmentConfiguration::getSMM() + "/models/techbox2.wrl");
   std::string USCar(ManagerEnvironmentConfiguration::getSMM() + "/models/USCar.wrl");
   std::string spaceGateR16(ManagerEnvironmentConfiguration::getSMM() + "/models/spaceGateR16.wrl");
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_dust+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_winter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/early_morning+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy3+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day2+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_deepsun+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_evening+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning2+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_noon+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_warp+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_Hubble_Nebula+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_easter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_hot_nebula+6.jpg" );
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_ice_field+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_lemon_lime+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_milk_chocolate+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_solar_bloom+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_thick_rb+6.jpg" );

   float ga = 0.1f; //Global Ambient Light level for this module
   ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
   WOLight* light = WOLight::New();
   light->isDirectionalLight( true );
   light->setPosition( Vector( 0, 0, 100 ) );
   //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
   //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
   light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
   light->setLabel( "Light" );
   worldLst->push_back( light );

   //Create the SkyBox
   WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
   wo->setPosition( Vector( 0,0,0 ) );
   wo->setLabel( "Sky Box" );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   worldLst->push_back( wo );

   ////Create the infinite grass plane (the floor)
   wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   wo->setPosition( Vector( 0, 0, 0 ) );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
   grassSkin.getMultiTextureSet().at( 0 )->setTextureRepeats( 5.0f );
   grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
   grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
   grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
   grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
   wo->setLabel( "Grass" );
   //worldLst->push_back( wo );

   // Add some objects to see the culling
   wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(0, 0, 0));
   worldLst->push_back(wo);
   wo->setLabel("Cube");

   wo = WO::New(wheeledCar, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(20, 20, 20));
   worldLst->push_back(wo);
   wo->setLabel("RCX Car");

   wo = WO::New(stone, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(-20, 20, 20));
   worldLst->push_back(wo);
   wo->setLabel("Stone");

   wo = WO::New(spaceAsteroid, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(20, -20, 20));
   worldLst->push_back(wo);
   wo->setLabel("Space Asteroid 1");

   wo = WO::New(spaceAsteroid2, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(20, 20, -20));
   worldLst->push_back(wo);
   wo->setLabel("Space Asteroid 2");

   wo = WO::New(spaceSatellite, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(-20, -20, 20));
   worldLst->push_back(wo);
   wo->setLabel("Space Satellite");

   wo = WO::New(techbox1, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(-20, 20, -20));
   worldLst->push_back(wo);
   wo->setLabel("Tech Box 1");

   wo = WO::New(techbox2, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(20, -20, -20));
   worldLst->push_back(wo);
   wo->setLabel("Tech Box 2");

   wo = WO::New(USCar, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(-20, -20, -20));
   worldLst->push_back(wo);
   wo->setLabel("US Car");

   wo = WO::New(spaceGateR16, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   wo->setPosition(Vector(0, 0, 50));
   worldLst->push_back(wo);
   wo->setLabel("Space Gate R16");

   // Create the camera frustum
   cameraFrustum = WO::New();
   cameraFrustum->setModel(MGLFrustum::New(cameraFrustum, 0.1f, 50, 135, 16.0f / 9.0f));
   cameraFrustum->setPosition(0, 0, 30);
   worldLst->push_back(cameraFrustum);
   cameraFrustum->isVisible = false;
   cameraFrustum->setLabel("Camera Frustum");

   // Create the non camera frustum
   nonCameraFrustum = WO::New();
   nonCameraFrustum->setModel(MGLFrustum::New(nonCameraFrustum, 10, 30, 100, 16.0f / 9.0f));
   nonCameraFrustum->setPosition(0, 0, 0);
   worldLst->push_back(nonCameraFrustum);
   nonCameraFrustum->isVisible = false;
   nonCameraFrustum->setLabel("Non Camera Frustum");

   std::string comicSans = ManagerEnvironmentConfiguration::getSMM() + "/fonts/COMIC.TTF";
   label = WOGUILabel::New(nullptr);
   label->setColor(0, 255, 0, 255);
   label->setFontSize(8); //font size is correlated with world size
   label->setPosition(Vector(0, 1, 0));
   label->setFontOrientation(FONT_ORIENTATION::foLEFT_TOP);
   label->setFontPath(comicSans);
   label->setLabel("Checker");
   worldLst->push_back(label);

   currentFrustum = cameraFrustum;
}

void Aftr::GLViewFrustumCulling::doFrustumCulling() {
	std::string rendered = "Rendered Objects:  ";
	// Frustum Culling Math
	MGLFrustum* frustumModel = static_cast<MGLFrustum*>(currentFrustum->getModel());
	float ratio = frustumModel->getAspectRatioWidthToHeight();
	// Since the GeometryFrustum takes the verticalFOV, need to calculate it
	// The formula was a derivation from the slides, using the relation between horizontal, vertical, and aspect ratio
	float verticalFOVdeg = static_cast<float>(RADtoDEG * 2 * atan((1.0f / ratio) * tan(DEGtoRAD * static_cast<double>(frustumModel->getHorzFOVDeg()) / 2.0)));
	AftrGeometryFrustum frustum(ratio, verticalFOVdeg,
		frustumModel->getNearPlane(), frustumModel->getFarPlane(), currentFrustum->getLookDirection(),
		currentFrustum->getNormalDirection(), currentFrustum->getPosition());

	// Get the normals and coefficients
	Vector normals[6];
	float coefs[6];
	for (size_t i = 0; i < 6; i++) {
		normals[i] = frustum.getPlaneNormal(static_cast<unsigned int>(i));
		coefs[i] = frustum.getPlaneCoef(static_cast<unsigned int>(i));
	}

	// Check if an object should be rendered
	for (size_t i = 0; i < worldLst->size(); i++) {
		// I want certain things to not be checked
		std::string label = worldLst->at(i)->getLabel();
		if ((label == "Sky Box") || (label == "Camera Frustum") || (label == "Non Camera Frustum") 
			|| (label == "Light") || (label == "Camera") || (label == "Checker")) continue;

		if (label == "RCX Car") worldLst->at(i)->rotateAboutGlobalZ(DEGtoRAD * 3); // Rotate the car

		// Store some data from this WO
		Mat4 displayMatrix = worldLst->at(i)->getDisplayMatrix();
		Vector objectPos = worldLst->at(i)->getPosition();
		// Calcluate the vertices of the bounding box for this object
		Vector extends = worldLst->at(i)->getModel()->getBoundingBox().getlxlylz();
		Vector points[8];
		points[0] = (displayMatrix * Vector(extends.x / 2.0f, extends.y / 2.0f, extends.z / 2.0f)) + objectPos;
		points[1] = (displayMatrix * Vector(extends.x / 2.0f, extends.y / 2.0f, extends.z / -2.0f)) + objectPos;
		points[2] = (displayMatrix * Vector(extends.x / 2.0f, extends.y / -2.0f, extends.z / 2.0f)) + objectPos;
		points[3] = (displayMatrix * Vector(extends.x / 2.0f, extends.y / -2.0f, extends.z / -2.0f)) + objectPos;
		points[4] = (displayMatrix * Vector(extends.x / -2.0f, extends.y / 2.0f, extends.z / 2.0f)) + objectPos;
		points[5] = (displayMatrix * Vector(extends.x / -2.0f, extends.y / 2.0f, extends.z / -2.0f)) + objectPos;
		points[6] = (displayMatrix * Vector(extends.x / -2.0f, extends.y / -2.0f, extends.z / 2.0f)) + objectPos;
		points[7] = (displayMatrix * Vector(extends.x / -2.0f, extends.y / -2.0f, extends.z / -2.0f)) + objectPos;
		// Check against each face of the frustum
		bool determined = false;
		for (size_t j = 0; (j < 6) && !determined; j++) {
			// If all 8 points are on the outside of any 1 side, it shouldn't have to render
			if ((normals[j].dotProduct(points[0]) >= coefs[j]) && (normals[j].dotProduct(points[1]) >= coefs[j]) &&
				(normals[j].dotProduct(points[2]) >= coefs[j]) && (normals[j].dotProduct(points[3]) >= coefs[j]) &&
				(normals[j].dotProduct(points[4]) >= coefs[j]) && (normals[j].dotProduct(points[5]) >= coefs[j]) &&
				(normals[j].dotProduct(points[6]) >= coefs[j]) && (normals[j].dotProduct(points[7]) >= coefs[j])) {
				worldLst->at(i)->isVisible = false;
				determined = true; // Can also break out of this for loop early
			}
		}
		// If all 8 points were not on the outside of any 1 side of the frustum, should still render it
		if (!determined) {
			worldLst->at(i)->isVisible = true;
			rendered = rendered + label + "  ";
		}
	}
	label->setText(rendered);
}
