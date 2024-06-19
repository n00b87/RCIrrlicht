// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#include "IrrCompileConfig.h"

#ifdef _IRR_COMPILE_WITH_WX_DEVICE_

#include "CIrrDeviceWx.h"
#include "IEventReceiver.h"
#include "irrList.h"
#include "os.h"
#include "CTimer.h"
#include "irrString.h"
#include "Keycodes.h"
#include "COSOperator.h"
#include <stdio.h>
#include <stdlib.h>
#include "SIrrCreationParameters.h"
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <wx/toplevel.h>

#ifdef _IRR_COMPILE_WITH_OGLES2_
#include "COGLES2Driver.h"
#endif // _IRR_COMPILE_WITH_OGLES2_

namespace irr
{
	namespace video
	{

		#ifdef _IRR_COMPILE_WITH_OPENGL_
		IVideoDriver* createOpenGLDriver(const SIrrlichtCreationParameters& params,
				io::IFileSystem* io, CIrrDeviceWx* device);
		#endif

		#ifdef _IRR_COMPILE_WITH_OGLES2_
		//IVideoDriver* createOGLES2Driver(const SIrrlichtCreationParameters& params,
		//		io::IFileSystem* io, CIrrDeviceWx* device);
		#endif
	} // end namespace video

} // end namespace irr


namespace irr
{

//! constructor
CIrrDeviceWx::CIrrDeviceWx(const SIrrlichtCreationParameters& param)
	: CIrrDeviceStub(param),
	parent((wxWindow*)param.WindowId), MouseX(0), MouseY(0), MouseButtonStates(0),
	Width(param.WindowSize.Width), Height(param.WindowSize.Height),
	Resizable(false), WindowMinimized(false)
{

    //os::Printer::Logger->setLogLevel(ELL_NONE);
	#ifdef _DEBUG
	setDebugName("CIrrDeviceWx");
	#endif

	// create keymap
	createKeyMap();

	// create window
	if (CreationParams.DriverType != video::EDT_NULL)
	{
		wxMessageBox(_("Create Window Start"));
		// create the window, only if we do not use the null device
		window = NULL;
		createWindow();
	}

	// create cursor control
	//CursorControl = new CCursorControl(this);

	// create driver
	createDriver();

	if (VideoDriver)
		createGUIAndScene();
}


//! destructor
CIrrDeviceWx::~CIrrDeviceWx()
{
#if defined(_IRR_COMPILE_WITH_JOYSTICK_EVENTS_)
	//Joysticks are closed when rcbasic shuts down

	//const u32 numJoysticks = Joysticks.size();
	//for (u32 i=0; i<numJoysticks; ++i)
	//	SDL_JoystickClose(Joysticks[i]);
#endif
	delete context;
	//SDL_Quit(); //This will be called when program ends
}


bool CIrrDeviceWx::createWindow()
{
    os::Printer::log("Create WxGLCanvas" );
	if ( Close )
		return false;

	wxGLAttributes attribs;

	if (CreationParams.DriverType == video::EDT_OPENGL || CreationParams.DriverType == video::EDT_OGLES2)
	{
		wxMessageBox(_("Get Attribs"));
		attribs = getAttributeList(CreationParams);

		if ( !window )
			window = new wxGLCanvas(parent, attribs, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE);

		if(window)
			wxMessageBox(_("Window has been created"));
		else
			wxMessageBox(_("Could not create default window"));


		if ( !window && CreationParams.AntiAlias>1)
		{
			while (--CreationParams.AntiAlias>1)
			{
				attribs = getAttributeList(CreationParams);
				window = new wxGLCanvas(parent, attribs, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE);
				if (window)
					break;
			}

			if ( !window )
			{
				CreationParams.AntiAlias = 0;
				attribs = getAttributeList(CreationParams);
				window = new wxGLCanvas(parent, attribs, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE);
				if (window)
					os::Printer::log("AntiAliasing disabled due to lack of support!" );
			}
		}
	}
	else
		return false;

	if ( !window && CreationParams.Doublebuffer)
	{
		// Try single buffer
		CreationParams.Doublebuffer = false;
		attribs = getAttributeList(CreationParams);
		window = new wxGLCanvas(parent, attribs, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE);
	}
	if ( !window )
	{
		os::Printer::log( "Could not initialize display!" );
		return false;
	}

	if(window)
		wxMessageBox(_("GLCanvas successfully created"));

	attributes = attribs;

	os::Printer::log( "WxWidgets Device Initialized**");

	return true;
}


//! create the driver
void CIrrDeviceWx::createDriver()
{
	switch(CreationParams.DriverType)
	{
	case video::EDT_OGLES2:
		//#ifdef _IRR_COMPILE_WITH_OGLES2_
		//VideoDriver = video::createOGLES2Driver(CreationParams, FileSystem, this);
		//#else
		os::Printer::log("No GLES2 support compiled in.", ELL_ERROR);
		//#endif
		break;

	case video::EDT_OPENGL:
		#ifdef _IRR_COMPILE_WITH_OPENGL_
		VideoDriver = video::createOpenGLDriver(CreationParams, FileSystem, this);
		#else
		os::Printer::log("No OpenGL support compiled in.", ELL_ERROR);
		#endif
		break;

	case video::EDT_NULL:
		VideoDriver = video::createNullDriver(FileSystem, CreationParams.WindowSize);
		break;

	default:
		os::Printer::log("Unable to create video driver of unknown type.", ELL_ERROR);
		break;
	}
}


//! runs the device. Returns false if device wants to be deleted
bool CIrrDeviceWx::run()
{
	os::Timer::tick();


	return !Close;
}

//! Activate any joysticks, and generate events for them.
bool CIrrDeviceWx::activateJoysticks(core::array<SJoystickInfo> & joystickInfo)
{
    //This is a place holder. RCBasic will init joysticks during its init phase.
	return true;
}



//! pause execution temporarily
void CIrrDeviceWx::yield()
{
	 	//wxMilliSleep(0);
	 	wxYield();
}


//! pause execution for a specified time
void CIrrDeviceWx::sleep(u32 timeMs, bool pauseTimer)
{
	const bool wasStopped = Timer ? Timer->isStopped() : true;
	if (pauseTimer && !wasStopped)
		Timer->stop();

	wxMilliSleep(timeMs);

	if (pauseTimer && !wasStopped)
		Timer->start();
}


//! sets the caption of the window
void CIrrDeviceWx::setWindowCaption(const wchar_t* text)
{
	core::stringc textc = text;
	//window is just the GL Canvas so I might need to change the constructor to accept the top level window to make this work
}


//! presents a surface in the client area
bool CIrrDeviceWx::present(video::IImage* surface, void* windowId, core::rect<s32>* srcClip)
{
	return true;
}


//! notifies the device that it should close itself
void CIrrDeviceWx::closeDevice()
{
	Close = true;
}


//! \return Pointer to a list with all video modes supported
video::IVideoModeList* CIrrDeviceWx::getVideoModeList()
{
	return VideoModeList;
}


//! Sets if the window should be resizable in windowed mode.
void CIrrDeviceWx::setResizable(bool resize)
{
}


//! Minimizes window if possible
void CIrrDeviceWx::minimizeWindow()
{
}


//! Maximize window
void CIrrDeviceWx::maximizeWindow()
{
	// do nothing
}


//! Restore original window size
void CIrrDeviceWx::restoreWindow()
{
	// do nothing
}

//! Get the position of the frame on-screen
core::position2di CIrrDeviceWx::getWindowPosition()
{
	return core::position2di(0,0);
}

//! returns if window is active. if not, nothing need to be drawn
bool CIrrDeviceWx::isWindowActive() const
{
	return true;
}


//! returns if window has focus.
bool CIrrDeviceWx::isWindowFocused() const
{
	return true;
}


//! returns if window is minimized.
bool CIrrDeviceWx::isWindowMinimized() const
{
	return WindowMinimized;
}


//! Set the current Gamma Value for the Display
bool CIrrDeviceWx::setGammaRamp( f32 red, f32 green, f32 blue, f32 brightness, f32 contrast )
{
	/*
	// todo: Gamma in SDL takes ints, what does Irrlicht use?
	return (SDL_SetGamma(red, green, blue) != -1);
	*/
	return false;
}

//! Get the current Gamma Value for the Display
bool CIrrDeviceWx::getGammaRamp( f32 &red, f32 &green, f32 &blue, f32 &brightness, f32 &contrast )
{
/*	brightness = 0.f;
	contrast = 0.f;
	return (SDL_GetGamma(&red, &green, &blue) != -1);*/
	return false;
}

//! returns color format of the window.
video::ECOLOR_FORMAT CIrrDeviceWx::getColorFormat() const
{
    return CIrrDeviceStub::getColorFormat();
}


void CIrrDeviceWx::createKeyMap()
{
	// I don't know if this is the best method  to create
	// the lookuptable, but I'll leave it like that until
	// I find a better version.

	KeyMap.reallocate(105);

	// buttons missing

	KeyMap.push_back(SKeyMap(WXK_BACK, KEY_BACK));
	KeyMap.push_back(SKeyMap(WXK_TAB, KEY_TAB));
	KeyMap.push_back(SKeyMap(WXK_CLEAR, KEY_CLEAR));
	KeyMap.push_back(SKeyMap(WXK_RETURN, KEY_RETURN));

	// combined modifiers missing

	KeyMap.push_back(SKeyMap(WXK_PAUSE, KEY_PAUSE));
	KeyMap.push_back(SKeyMap(WXK_CAPITAL, KEY_CAPITAL));

	// asian letter keys missing

	KeyMap.push_back(SKeyMap(WXK_ESCAPE, KEY_ESCAPE));

	// asian letter keys missing

	KeyMap.push_back(SKeyMap(WXK_SPACE, KEY_SPACE));
	KeyMap.push_back(SKeyMap(WXK_PAGEUP, KEY_PRIOR));
	KeyMap.push_back(SKeyMap(WXK_PAGEDOWN, KEY_NEXT));
	KeyMap.push_back(SKeyMap(WXK_END, KEY_END));
	KeyMap.push_back(SKeyMap(WXK_HOME, KEY_HOME));
	KeyMap.push_back(SKeyMap(WXK_LEFT, KEY_LEFT));
	KeyMap.push_back(SKeyMap(WXK_UP, KEY_UP));
	KeyMap.push_back(SKeyMap(WXK_RIGHT, KEY_RIGHT));
	KeyMap.push_back(SKeyMap(WXK_DOWN, KEY_DOWN));

	// select missing
	KeyMap.push_back(SKeyMap(WXK_PRINT, KEY_PRINT));
	// execute missing
	KeyMap.push_back(SKeyMap(WXK_PRINT, KEY_SNAPSHOT));

	KeyMap.push_back(SKeyMap(WXK_INSERT, KEY_INSERT));
	KeyMap.push_back(SKeyMap(WXK_DELETE, KEY_DELETE));
	KeyMap.push_back(SKeyMap(WXK_HELP, KEY_HELP));

	KeyMap.push_back(SKeyMap(48, KEY_KEY_0));
	KeyMap.push_back(SKeyMap(49, KEY_KEY_1));
	KeyMap.push_back(SKeyMap(50, KEY_KEY_2));
	KeyMap.push_back(SKeyMap(51, KEY_KEY_3));
	KeyMap.push_back(SKeyMap(52, KEY_KEY_4));
	KeyMap.push_back(SKeyMap(53, KEY_KEY_5));
	KeyMap.push_back(SKeyMap(54, KEY_KEY_6));
	KeyMap.push_back(SKeyMap(55, KEY_KEY_7));
	KeyMap.push_back(SKeyMap(56, KEY_KEY_8));
	KeyMap.push_back(SKeyMap(57, KEY_KEY_9));

	KeyMap.push_back(SKeyMap(65, KEY_KEY_A));
	KeyMap.push_back(SKeyMap(66, KEY_KEY_B));
	KeyMap.push_back(SKeyMap(67, KEY_KEY_C));
	KeyMap.push_back(SKeyMap(68, KEY_KEY_D));
	KeyMap.push_back(SKeyMap(69, KEY_KEY_E));
	KeyMap.push_back(SKeyMap(70, KEY_KEY_F));
	KeyMap.push_back(SKeyMap(71, KEY_KEY_G));
	KeyMap.push_back(SKeyMap(72, KEY_KEY_H));
	KeyMap.push_back(SKeyMap(73, KEY_KEY_I));
	KeyMap.push_back(SKeyMap(74, KEY_KEY_J));
	KeyMap.push_back(SKeyMap(75, KEY_KEY_K));
	KeyMap.push_back(SKeyMap(76, KEY_KEY_L));
	KeyMap.push_back(SKeyMap(77, KEY_KEY_M));
	KeyMap.push_back(SKeyMap(78, KEY_KEY_N));
	KeyMap.push_back(SKeyMap(79, KEY_KEY_O));
	KeyMap.push_back(SKeyMap(80, KEY_KEY_P));
	KeyMap.push_back(SKeyMap(81, KEY_KEY_Q));
	KeyMap.push_back(SKeyMap(82, KEY_KEY_R));
	KeyMap.push_back(SKeyMap(83, KEY_KEY_S));
	KeyMap.push_back(SKeyMap(84, KEY_KEY_T));
	KeyMap.push_back(SKeyMap(85, KEY_KEY_U));
	KeyMap.push_back(SKeyMap(86, KEY_KEY_V));
	KeyMap.push_back(SKeyMap(87, KEY_KEY_W));
	KeyMap.push_back(SKeyMap(88, KEY_KEY_X));
	KeyMap.push_back(SKeyMap(89, KEY_KEY_Y));
	KeyMap.push_back(SKeyMap(90, KEY_KEY_Z));

        // TODO:
	//KeyMap.push_back(SKeyMap(WXK_LSUPER, KEY_LWIN));
        // TODO:
	//KeyMap.push_back(SKeyMap(WXK_RSUPER, KEY_RWIN));
	// apps missing
	//KeyMap.push_back(SKeyMap(WXK_POWER, KEY_SLEEP)); //??

	KeyMap.push_back(SKeyMap(WXK_NUMPAD0, KEY_NUMPAD0));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD1, KEY_NUMPAD1));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD2, KEY_NUMPAD2));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD3, KEY_NUMPAD3));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD4, KEY_NUMPAD4));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD5, KEY_NUMPAD5));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD6, KEY_NUMPAD6));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD7, KEY_NUMPAD7));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD8, KEY_NUMPAD8));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD9, KEY_NUMPAD9));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD_MULTIPLY, KEY_MULTIPLY));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD_ADD, KEY_ADD));
//	KeyMap.push_back(SKeyMap(WXK_KP_, KEY_SEPARATOR));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD_SUBTRACT, KEY_SUBTRACT));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD_DECIMAL, KEY_DECIMAL));
	KeyMap.push_back(SKeyMap(WXK_NUMPAD_DIVIDE, KEY_DIVIDE));

	KeyMap.push_back(SKeyMap(WXK_F1,  KEY_F1));
	KeyMap.push_back(SKeyMap(WXK_F2,  KEY_F2));
	KeyMap.push_back(SKeyMap(WXK_F3,  KEY_F3));
	KeyMap.push_back(SKeyMap(WXK_F4,  KEY_F4));
	KeyMap.push_back(SKeyMap(WXK_F5,  KEY_F5));
	KeyMap.push_back(SKeyMap(WXK_F6,  KEY_F6));
	KeyMap.push_back(SKeyMap(WXK_F7,  KEY_F7));
	KeyMap.push_back(SKeyMap(WXK_F8,  KEY_F8));
	KeyMap.push_back(SKeyMap(WXK_F9,  KEY_F9));
	KeyMap.push_back(SKeyMap(WXK_F10, KEY_F10));
	KeyMap.push_back(SKeyMap(WXK_F11, KEY_F11));
	KeyMap.push_back(SKeyMap(WXK_F12, KEY_F12));
	KeyMap.push_back(SKeyMap(WXK_F13, KEY_F13));
	KeyMap.push_back(SKeyMap(WXK_F14, KEY_F14));
	KeyMap.push_back(SKeyMap(WXK_F15, KEY_F15));
	// no higher F-keys

        // TODO:
	//KeyMap.push_back(SKeyMap(WXK_NUMLOCK, KEY_NUMLOCK));
	KeyMap.push_back(SKeyMap(WXK_SCROLL, KEY_SCROLL));
	KeyMap.push_back(SKeyMap(WXK_SHIFT, KEY_LSHIFT));
	KeyMap.push_back(SKeyMap(WXK_SHIFT, KEY_RSHIFT));
	KeyMap.push_back(SKeyMap(WXK_CONTROL,  KEY_LCONTROL));
	KeyMap.push_back(SKeyMap(WXK_CONTROL,  KEY_RCONTROL));
	KeyMap.push_back(SKeyMap(WXK_ALT,  KEY_LMENU));
	KeyMap.push_back(SKeyMap(WXK_ALT,  KEY_RMENU));

	KeyMap.push_back(SKeyMap(WXK_ADD,   KEY_PLUS));
	KeyMap.push_back(SKeyMap(WXK_SEPARATOR,  KEY_COMMA));
	KeyMap.push_back(SKeyMap(WXK_SUBTRACT,  KEY_MINUS));
	KeyMap.push_back(SKeyMap(WXK_DECIMAL, KEY_PERIOD));

	// some special keys missing

	KeyMap.sort();
}

} // end namespace irr

#endif // _IRR_COMPILE_WITH_SDL_DEVICE_

