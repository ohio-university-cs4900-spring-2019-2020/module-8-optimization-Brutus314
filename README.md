# module-8-optimization-Brutus314
module-8-optimization-Brutus314 created by GitHub Classroom

This module implements some rendering optimization in the form of frustum culling.
This module uses 2 frustums. You will start in "camera mode", so the current frustum
is your camera and is everything you would expect. Pressing '1' will toggle this mode.
While you are not in "camera mode", the frustum will be centered at the world origin.
You can see this frustum while it is active. It will follow your look direction in the 
X and Y directions, but will be Z-locked to 0 (will not move up and down). Pressing '2' 
will toggle freezing this frustum, so that it will not move. This works whether or not 
it you are in "camera mode", and it also persists through toggling "camera mode". 
There are several objects near the origin to apply this concept to.

The origin frustum also has a near plane that is not very near the origin, unlike the camera.
You'll notice the cube is never rendered when "camera mode" is off. 

In the top-right corner of the screen, you will see a list of all objects currently rendered.