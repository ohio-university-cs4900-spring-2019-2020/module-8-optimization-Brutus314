#pragma once

#include "GLView.h"
#include "WOGUILabel.h"

namespace Aftr
{
   class Camera;

/**
   \class GLViewFrustumCulling
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewFrustumCulling : public GLView
{
public:
   static GLViewFrustumCulling* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewFrustumCulling();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );

protected:
   GLViewFrustumCulling( const std::vector< std::string >& args );
   virtual void onCreate();   

   void doFrustumCulling();

   WO* cameraFrustum;
   WO* nonCameraFrustum;
   WO* currentFrustum;
   WOGUILabel* label;

   bool doRotate;
};

/** \} */

} //namespace Aftr
