/** Example 001 HelloWorld

This tutorial shows how to set up the IDE for using the Irrlicht Engine and how
to write a simple HelloWorld program with it. The program will show how to use
the basics of the VideoDriver, the GUIEnvironment, and the SceneManager.
Microsoft Visual Studio is used as an IDE, but you will also be able to
understand everything if you are using a different one or even another
operating system than Windows.

You have to include the header file <irrlicht.h> in order to use the engine. The
header file can be found in the Irrlicht Engine SDK directory \c include. To let
the compiler find this header file, the directory where it is located has to be
added in your project as include path. This is different for every IDE and
compiler you use. Let's explain shortly how to do this in Visual Studio 2010:

- In Visual Studio 2010 select the Menu Project -> Properties. Select the
  "C/C++" - "General" option, and select the "Additional Include Directories".
  Add the \c include directory of the Irrlicht engine folder to the list of
  directories. Now the compiler will find the irrlicht.h header file. We also
  need the irrlicht.lib to be found, so select "Linker" - "General" and
  add the \c lib/Win64-visualStudio or \c lib/Win32-visualStudio directory
  to "Additional Library Directories". Which of the 2 Irrlicht versions you
  chose depends on the target platform for your application (win32 or x64).
  In your project properties you can see what your active solution platform
  is, you can use the same one for Irrlicht.

To be able to use the Irrlicht.DLL file, we need to link with the Irrlicht.lib.
In most IDE's you have to add irrlicht.lib (or irrlicht.a or irrlicht.so on
Linux) to your Linker input files.

For VisualStudio we can be lazy and use the pragma comment lib.
We also want to get rid of the console window, which pops up when starting a
program with main() (instead of WinMain). This is done by the second pragma.
We could also use the WinMain method, though losing platform independence then.
*/
#ifdef _MSC_VER
#pragma comment(lib, "Irrlicht.lib")
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

/*
That's it. With your IDE set up like this, you will now be able to develop
applications with the Irrlicht Engine.

Lets start!

After we have set up the IDE, the compiler will know where to find the Irrlicht
Engine header files so we can include it now in our code.
*/
#include <irrlicht.h>
#include <SDL2/SDL.h>
#include <iostream>
#include <vector>

/*
That header just adds the getExampleMediaPath tool-functions to help locating
the media we need. More about that later below.
*/
#include "exampleHelper.h"

/*
In the Irrlicht Engine, everything can be found in the namespace 'irr'. So if
you want to use a class of the engine, you have to write irr:: before the name
of the class. For example to use the IrrlichtDevice write: irr::IrrlichtDevice.
To get rid of the irr:: in front of the name of every class, we tell the
compiler that we use that namespace from now on, and we will not have to write
irr:: anymore.
Note that you never should do that in headers - otherwise you will pollute the
namespace of every file including such a header. So in headers always write
out the full names including all namespaces.
*/
using namespace irr;

/*
There are 5 sub namespaces in the Irrlicht Engine. Take a look at them, you can
read a detailed description of them in the documentation by clicking on the top
menu item 'Namespace List' or by using this link:
http://irrlicht.sourceforge.net/docu/namespaces.html
Like the irr namespace, we do not want these 5 sub namespaces now, to keep this
example simple. Hence, we tell the compiler again that we do not want always to
write their names.
*/
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


// Function to calculate a point on a cubic Bezier curve
vector3df bezierPoint(const vector3df& p0, const vector3df& p1, const vector3df& p2, const vector3df& p3, float t) {
    float u = 1.0f - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;

    vector3df p = uuu * p0; // (1-t)^3 * p0
    p += 3 * uu * t * p1;   // 3 * (1-t)^2 * t * p1
    p += 3 * u * tt * p2;   // 3 * (1-t) * t^2 * p2
    p += ttt * p3;          // t^3 * p3

    return p;
}

// Function to draw a Bezier curve
void drawBezierCurve(IVideoDriver* driver, const vector3df& p0, const vector3df& p1, const vector3df& p2, const vector3df& p3, SColor color, int segments) {
    std::vector<vector3df> points;

    for (int i = 0; i <= segments; ++i) {
        float t = static_cast<float>(i) / segments;
        points.push_back(bezierPoint(p0, p1, p2, p3, t));
    }

    for (size_t i = 0; i < points.size() - 1; ++i) {
        driver->draw3DLine(points[i], points[i + 1], color);
    }
}


/*
This is the main method. We can now use main() on every platform.
*/
int main()
{
	/*
	The most important function of the engine is the createDevice()
	function. The IrrlichtDevice is created by it, which is the root
	object for doing anything with the engine. createDevice() has the
	following parameters:

	- driverType: Type of the video driver. This can currently be the Null-device,
	   one of the two software renderers, D3D9, or OpenGL. In this
	   example we use EDT_BURNINGSVIDEO, but to try out, you might want to
	   change it to EDT_SOFTWARE, EDT_NULL, EDT_DIRECT3D9, or EDT_OPENGL.
	   Generally you will want to use OpenGL or Direct3D as they are
	   using your graphic card for calculations instead of the CPU and
	   are way faster (and usually better looking). We just use one of the
	   software renderers here as it even works when your graphic card driver
	   isn't set up for 3d support.

	- windowSize: Size of the Window or screen in FullScreenMode to be
	   created. In this example we use 640x480.

	- bits: Amount of color bits per pixel. This should be 16 or 32. The
	   parameter is often ignored when running in windowed mode. More
	   commonly you would chose 32 bit, again we're just playing it safe.

	- fullscreen: Specifies if we want the device to run in fullscreen mode
	   or windowed.

	- stencilbuffer: Specifies if we want to use the stencil buffer (you
		need it for drawing shadows).

	- vsync: Specifies if we want to have vsync enabled, this is only useful
	   in fullscreen mode.

	- eventReceiver: An object to receive events. We do not want to use this
	   parameter here, and set it to 0.

	Always check the return value to cope with unsupported drivers,
	dimensions, etc.
	*/

	#ifdef RC_WEB
    if(SDL_Init(SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_NOPARACHUTE) < 0)
    #else
    //if(SDL_Init(SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC | SDL_INIT_SENSOR | SDL_INIT_NOPARACHUTE) < 0)
    if(SDL_Init(SDL_INIT_EVENTS | SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC | SDL_INIT_SENSOR | SDL_INIT_NOPARACHUTE) < 0) //Audio causes init to fail on Fedora40 so I am leaving it out for now
    #endif
    {
        //os::Printer::log("SDL_Init Error: ", SDL_GetError());
        std::cout << "No DICE" << std::endl;
        return 0;
    }

    SDL_Window* window = SDL_CreateWindow("OGLES2 Test Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);

	SIrrlichtCreationParameters p;
		p.DriverType = EDT_OGLES2;
		p.WindowSize = dimension2d<u32>(640, 480);
		p.Bits = 32;
		p.Fullscreen = false;
		p.Stencilbuffer = false;
		p.Vsync = false;
		p.EventReceiver = 0;
		p.DeviceType = EIDT_SDL;
		p.WindowId = window;
	IrrlichtDevice *device = createDeviceEx(p);
		//createDevice( video::EDT_OGLES2, dimension2d<u32>(640, 480), 16,
			//false, false, false, 0);

	if (!device)
		return 1;

	/*
	Set the caption of the window to some nice text. Note that there is an
	'L' in front of the string. The Irrlicht Engine uses wide character
	strings when displaying text.
	*/
	//device->setWindowCaption(L"Hello World! - Irrlicht Engine Demo");

	/*
	Get a pointer to the VideoDriver, the SceneManager and the graphical
	user interface environment, so that we do not always have to write
	device->getVideoDriver(), device->getSceneManager(), or
	device->getGUIEnvironment().
	*/
	IVideoDriver* driver = device->getVideoDriver();
	ISceneManager* smgr = device->getSceneManager();
	IGUIEnvironment* guienv = device->getGUIEnvironment();

	/*
	We add a hello world label to the window, using the GUI environment.
	The text is placed at the position (10,10) as top left corner and
	(260,22) as lower right corner.
	*/
	guienv->addStaticText(L"Hello World! This is Irrlicht with the burnings software renderer!",
		rect<s32>(10,10,260,22), true);

	/*
	Get a media path dedicated for your platform. Finding media files for your
	applications can be tricky. First you have 2 options - working with relative
	paths or working with absolute paths.

	On Windows a common solution is that your installer will write a key into
	the registry with the absolute path of wherever the user installed the
	media. And in your application you read out that key from the registry.
	On Linux a common solution is to use config file which is placed in some
	fixed location (for example in a . file/folder in the user home).

	But you can also work with relative paths - which is what we do here. There
	is a slight complication with relative paths as they are relative to your
	current working directory. And that depends on the way your application is
	started and it might change inside your application. But mostly it will be
	set to your executable on start so you can ignore that problem while
	developing.

	When inside VisualStudio the current working directory is set to your
	project files location unless you overwrite Project properties - Debugging
	- Working Directory. In Irrlicht examples the media folder is on most
	platforms ../../media which works for the examples as it's relative to our
	project files as well as to the binary (.exe) files.

	Whatever you chose to find your base-folder for media - wrap it with some
	function and then you can improve the code to locate the media later on.
	*/
	const io::path mediaPath = getExampleMediaPath();

	/*
	To show something interesting, we load a Quake 2 model and display it.
	We get the Mesh from the Scene Manager with getMesh() and add a SceneNode
	to display the mesh with addAnimatedMeshSceneNode(). Check the return value
	of getMesh() to become aware of loading problems and other errors.

	Instead of writing the filename sydney.md2, it would also be possible
	to load a Maya object file (.obj), a complete Quake3 map (.bsp) or any
	other supported file format. By the way, that cool Quake 2 model
	called sydney was modeled by Brian Collins.
	*/
	IAnimatedMesh* mesh = smgr->getMesh(mediaPath + "sydney.md2");
	if (!mesh)
	{
		device->drop();
		return 1;
	}
	IAnimatedMeshSceneNode* node = smgr->addAnimatedMeshSceneNode( mesh );

	/*
	To let the mesh look a little bit nicer, we change its material. We
	disable lighting because we do not have a dynamic light in here, and
	the mesh would be totally black otherwise. Then we set the frame loop,
	such that the predefined STAND animation is used. And last, we apply a
	texture to the mesh. Without it the mesh would be drawn using only a
	color.
	*/
	if (node)
	{
		node->setMaterialFlag(EMF_LIGHTING, false);
		node->setMD2Animation(scene::EMAT_STAND);
		node->setMaterialTexture( 0, driver->getTexture(mediaPath + "sydney.bmp") );
	}

	/*
	To look at the mesh, we place a camera into 3d space at the position
	(0, 30, -40). The camera looks from there to (0,5,0), which is
	approximately the place where our md2 model is.
	*/
	smgr->addCameraSceneNode(0, vector3df(0,30,-40), vector3df(0,5,0));

	/*
	OK, now we have set up the scene, lets draw everything: We run the
	device in a while() loop, until the device does not want to run any
	more. This would be when the user closes the window or presses ALT+F4
	(or whatever keycode closes a window on your OS).
	*/
	SDL_Event event;

	bool quit = false;

	while(device->run() && (!quit))
	{
        while(SDL_PollEvent(&event))
        {
            switch(event.type)
            {
                case SDL_WINDOWEVENT:
                    if(event.window.event == SDL_WINDOWEVENT_RESIZED)
                    {

                    }
                    else if(event.window.event == SDL_WINDOWEVENT_CLOSE)
                    {
                        if(SDL_QuitRequested() != 0)
                        {
                            //SDL_PumpEvents();
                            SDL_FlushEvent(SDL_QUIT);
                            quit = true;
                        }
                        std::cout << "SDL QUIT" << std::endl;
                    }
                    break;
            }
        }
		/*
		Anything can be drawn between a beginScene() and an endScene()
		call. The beginScene() call clears the screen with a color and
		the depth buffer, if desired. Then we let the Scene Manager and
		the GUI Environment draw their content. With the endScene()
		call everything is presented on the screen.
		*/
		driver->beginScene(ECBF_COLOR | ECBF_DEPTH, SColor(255,100,101,140));

		smgr->drawAll();

		vector3df p0(0, 0, 0);
		vector3df p1(10, 30, 0);
		vector3df p2(20, -30, 0);
		vector3df p3(30, 0, 0);

		drawBezierCurve(driver, p0, p1, p2, p3, SColor(255, 255, 255, 255), 100);

		guienv->drawAll();

		driver->endScene();
	}

	/*
	After we are done with the render loop, we have to delete the Irrlicht
	Device created before with createDevice(). In the Irrlicht Engine, you
	have to delete all objects you created with a method or function which
	starts with 'create'. The object is simply deleted by calling ->drop().
	See the documentation at irr::IReferenceCounted::drop() for more
	information.
	*/
	SDL_DestroyWindow(window);
	SDL_Quit();
	device->drop();

	return 0;
}

/*
That's it. Compile and run.
**/
