#ifndef RC_GFX_INCLUDED
#define RC_GFX_INCLUDED

#include <SDL2/SDL.h>
#include <irrlicht.h>
#include <iostream>

using namespace irr;

using namespace core;
using namespace video;
using namespace scene;

struct SDLKeyMap
{
    SDLKeyMap() {}
    SDLKeyMap(s32 x11, s32 win32)
        : SDLKey(x11), Win32Key(win32)
    {
    }

    s32 SDLKey;
    s32 Win32Key;

    bool operator<(const SDLKeyMap& o) const
    {
        return SDLKey<o.SDLKey;
    }
};

core::array<SDLKeyMap> KeyMap;

void createKeyMap()
{
	// I don't know if this is the best method  to create
	// the lookuptable, but I'll leave it like that until
	// I find a better version.

	KeyMap.reallocate(105);

	// buttons missing

	KeyMap.push_back(SDLKeyMap(SDLK_BACKSPACE, irr::EKEY_CODE::KEY_BACK));
	KeyMap.push_back(SDLKeyMap(SDLK_TAB, irr::EKEY_CODE::KEY_TAB));
	KeyMap.push_back(SDLKeyMap(SDLK_CLEAR, irr::EKEY_CODE::KEY_CLEAR));
	KeyMap.push_back(SDLKeyMap(SDLK_RETURN, irr::EKEY_CODE::KEY_RETURN));

	// combined modifiers missing

	KeyMap.push_back(SDLKeyMap(SDLK_PAUSE, irr::EKEY_CODE::KEY_PAUSE));
	KeyMap.push_back(SDLKeyMap(SDLK_CAPSLOCK, irr::EKEY_CODE::KEY_CAPITAL));

	// asian letter keys missing

	KeyMap.push_back(SDLKeyMap(SDLK_ESCAPE, irr::EKEY_CODE::KEY_ESCAPE));

	// asian letter keys missing

	KeyMap.push_back(SDLKeyMap(SDLK_SPACE, irr::EKEY_CODE::KEY_SPACE));
	KeyMap.push_back(SDLKeyMap(SDLK_PAGEUP, irr::EKEY_CODE::KEY_PRIOR));
	KeyMap.push_back(SDLKeyMap(SDLK_PAGEDOWN, irr::EKEY_CODE::KEY_NEXT));
	KeyMap.push_back(SDLKeyMap(SDLK_END, irr::EKEY_CODE::KEY_END));
	KeyMap.push_back(SDLKeyMap(SDLK_HOME, irr::EKEY_CODE::KEY_HOME));
	KeyMap.push_back(SDLKeyMap(SDLK_LEFT, irr::EKEY_CODE::KEY_LEFT));
	KeyMap.push_back(SDLKeyMap(SDLK_UP, irr::EKEY_CODE::KEY_UP));
	KeyMap.push_back(SDLKeyMap(SDLK_RIGHT, irr::EKEY_CODE::KEY_RIGHT));
	KeyMap.push_back(SDLKeyMap(SDLK_DOWN, irr::EKEY_CODE::KEY_DOWN));

	// select missing
	KeyMap.push_back(SDLKeyMap(SDLK_PRINTSCREEN, irr::EKEY_CODE::KEY_PRINT));
	// execute missing
	KeyMap.push_back(SDLKeyMap(SDLK_PRINTSCREEN, irr::EKEY_CODE::KEY_SNAPSHOT));

	KeyMap.push_back(SDLKeyMap(SDLK_INSERT, irr::EKEY_CODE::KEY_INSERT));
	KeyMap.push_back(SDLKeyMap(SDLK_DELETE, irr::EKEY_CODE::KEY_DELETE));
	KeyMap.push_back(SDLKeyMap(SDLK_HELP, irr::EKEY_CODE::KEY_HELP));

	KeyMap.push_back(SDLKeyMap(SDLK_0, irr::EKEY_CODE::KEY_KEY_0));
	KeyMap.push_back(SDLKeyMap(SDLK_1, irr::EKEY_CODE::KEY_KEY_1));
	KeyMap.push_back(SDLKeyMap(SDLK_2, irr::EKEY_CODE::KEY_KEY_2));
	KeyMap.push_back(SDLKeyMap(SDLK_3, irr::EKEY_CODE::KEY_KEY_3));
	KeyMap.push_back(SDLKeyMap(SDLK_4, irr::EKEY_CODE::KEY_KEY_4));
	KeyMap.push_back(SDLKeyMap(SDLK_5, irr::EKEY_CODE::KEY_KEY_5));
	KeyMap.push_back(SDLKeyMap(SDLK_6, irr::EKEY_CODE::KEY_KEY_6));
	KeyMap.push_back(SDLKeyMap(SDLK_7, irr::EKEY_CODE::KEY_KEY_7));
	KeyMap.push_back(SDLKeyMap(SDLK_8, irr::EKEY_CODE::KEY_KEY_8));
	KeyMap.push_back(SDLKeyMap(SDLK_9, irr::EKEY_CODE::KEY_KEY_9));

	KeyMap.push_back(SDLKeyMap(SDLK_a, irr::EKEY_CODE::KEY_KEY_A));
	KeyMap.push_back(SDLKeyMap(SDLK_b, irr::EKEY_CODE::KEY_KEY_B));
	KeyMap.push_back(SDLKeyMap(SDLK_c, irr::EKEY_CODE::KEY_KEY_C));
	KeyMap.push_back(SDLKeyMap(SDLK_d, irr::EKEY_CODE::KEY_KEY_D));
	KeyMap.push_back(SDLKeyMap(SDLK_e, irr::EKEY_CODE::KEY_KEY_E));
	KeyMap.push_back(SDLKeyMap(SDLK_f, irr::EKEY_CODE::KEY_KEY_F));
	KeyMap.push_back(SDLKeyMap(SDLK_g, irr::EKEY_CODE::KEY_KEY_G));
	KeyMap.push_back(SDLKeyMap(SDLK_h, irr::EKEY_CODE::KEY_KEY_H));
	KeyMap.push_back(SDLKeyMap(SDLK_i, irr::EKEY_CODE::KEY_KEY_I));
	KeyMap.push_back(SDLKeyMap(SDLK_j, irr::EKEY_CODE::KEY_KEY_J));
	KeyMap.push_back(SDLKeyMap(SDLK_k, irr::EKEY_CODE::KEY_KEY_K));
	KeyMap.push_back(SDLKeyMap(SDLK_l, irr::EKEY_CODE::KEY_KEY_L));
	KeyMap.push_back(SDLKeyMap(SDLK_m, irr::EKEY_CODE::KEY_KEY_M));
	KeyMap.push_back(SDLKeyMap(SDLK_n, irr::EKEY_CODE::KEY_KEY_N));
	KeyMap.push_back(SDLKeyMap(SDLK_o, irr::EKEY_CODE::KEY_KEY_O));
	KeyMap.push_back(SDLKeyMap(SDLK_p, irr::EKEY_CODE::KEY_KEY_P));
	KeyMap.push_back(SDLKeyMap(SDLK_q, irr::EKEY_CODE::KEY_KEY_Q));
	KeyMap.push_back(SDLKeyMap(SDLK_r, irr::EKEY_CODE::KEY_KEY_R));
	KeyMap.push_back(SDLKeyMap(SDLK_s, irr::EKEY_CODE::KEY_KEY_S));
	KeyMap.push_back(SDLKeyMap(SDLK_t, irr::EKEY_CODE::KEY_KEY_T));
	KeyMap.push_back(SDLKeyMap(SDLK_u, irr::EKEY_CODE::KEY_KEY_U));
	KeyMap.push_back(SDLKeyMap(SDLK_v, irr::EKEY_CODE::KEY_KEY_V));
	KeyMap.push_back(SDLKeyMap(SDLK_w, irr::EKEY_CODE::KEY_KEY_W));
	KeyMap.push_back(SDLKeyMap(SDLK_x, irr::EKEY_CODE::KEY_KEY_X));
	KeyMap.push_back(SDLKeyMap(SDLK_y, irr::EKEY_CODE::KEY_KEY_Y));
	KeyMap.push_back(SDLKeyMap(SDLK_z, irr::EKEY_CODE::KEY_KEY_Z));

        // TODO:
	//KeyMap.push_back(SDLKeyMap(SDLK_LSUPER, KEY_LWIN));
        // TODO:
	//KeyMap.push_back(SDLKeyMap(SDLK_RSUPER, KEY_RWIN));
	// apps missing
	KeyMap.push_back(SDLKeyMap(SDLK_POWER, irr::EKEY_CODE::KEY_SLEEP)); //??

	KeyMap.push_back(SDLKeyMap(SDLK_KP_0, irr::EKEY_CODE::KEY_NUMPAD0));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_1, irr::EKEY_CODE::KEY_NUMPAD1));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_2, irr::EKEY_CODE::KEY_NUMPAD2));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_3, irr::EKEY_CODE::KEY_NUMPAD3));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_4, irr::EKEY_CODE::KEY_NUMPAD4));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_5, irr::EKEY_CODE::KEY_NUMPAD5));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_6, irr::EKEY_CODE::KEY_NUMPAD6));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_7, irr::EKEY_CODE::KEY_NUMPAD7));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_8, irr::EKEY_CODE::KEY_NUMPAD8));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_9, irr::EKEY_CODE::KEY_NUMPAD9));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_MULTIPLY, irr::EKEY_CODE::KEY_MULTIPLY));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_PLUS, irr::EKEY_CODE::KEY_ADD));
//	KeyMap.push_back(SDLKeyMap(SDLK_KP_, KEY_SEPARATOR));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_MINUS, irr::EKEY_CODE::KEY_SUBTRACT));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_PERIOD, irr::EKEY_CODE::KEY_DECIMAL));
	KeyMap.push_back(SDLKeyMap(SDLK_KP_DIVIDE, irr::EKEY_CODE::KEY_DIVIDE));

	KeyMap.push_back(SDLKeyMap(SDLK_F1,  irr::EKEY_CODE::KEY_F1));
	KeyMap.push_back(SDLKeyMap(SDLK_F2,  irr::EKEY_CODE::KEY_F2));
	KeyMap.push_back(SDLKeyMap(SDLK_F3,  irr::EKEY_CODE::KEY_F3));
	KeyMap.push_back(SDLKeyMap(SDLK_F4,  irr::EKEY_CODE::KEY_F4));
	KeyMap.push_back(SDLKeyMap(SDLK_F5,  irr::EKEY_CODE::KEY_F5));
	KeyMap.push_back(SDLKeyMap(SDLK_F6,  irr::EKEY_CODE::KEY_F6));
	KeyMap.push_back(SDLKeyMap(SDLK_F7,  irr::EKEY_CODE::KEY_F7));
	KeyMap.push_back(SDLKeyMap(SDLK_F8,  irr::EKEY_CODE::KEY_F8));
	KeyMap.push_back(SDLKeyMap(SDLK_F9,  irr::EKEY_CODE::KEY_F9));
	KeyMap.push_back(SDLKeyMap(SDLK_F10, irr::EKEY_CODE::KEY_F10));
	KeyMap.push_back(SDLKeyMap(SDLK_F11, irr::EKEY_CODE::KEY_F11));
	KeyMap.push_back(SDLKeyMap(SDLK_F12, irr::EKEY_CODE::KEY_F12));
	KeyMap.push_back(SDLKeyMap(SDLK_F13, irr::EKEY_CODE::KEY_F13));
	KeyMap.push_back(SDLKeyMap(SDLK_F14, irr::EKEY_CODE::KEY_F14));
	KeyMap.push_back(SDLKeyMap(SDLK_F15, irr::EKEY_CODE::KEY_F15));
	// no higher F-keys

        // TODO:
	//KeyMap.push_back(SDLKeyMap(SDLK_NUMLOCK, KEY_NUMLOCK));
	KeyMap.push_back(SDLKeyMap(SDLK_SCROLLLOCK, irr::EKEY_CODE::KEY_SCROLL));
	KeyMap.push_back(SDLKeyMap(SDLK_LSHIFT, irr::EKEY_CODE::KEY_LSHIFT));
	KeyMap.push_back(SDLKeyMap(SDLK_RSHIFT, irr::EKEY_CODE::KEY_RSHIFT));
	KeyMap.push_back(SDLKeyMap(SDLK_LCTRL,  irr::EKEY_CODE::KEY_LCONTROL));
	KeyMap.push_back(SDLKeyMap(SDLK_RCTRL,  irr::EKEY_CODE::KEY_RCONTROL));
	KeyMap.push_back(SDLKeyMap(SDLK_LALT,  irr::EKEY_CODE::KEY_LMENU));
	KeyMap.push_back(SDLKeyMap(SDLK_RALT,  irr::EKEY_CODE::KEY_RMENU));

	KeyMap.push_back(SDLKeyMap(SDLK_PLUS,   irr::EKEY_CODE::KEY_PLUS));
	KeyMap.push_back(SDLKeyMap(SDLK_COMMA,  irr::EKEY_CODE::KEY_COMMA));
	KeyMap.push_back(SDLKeyMap(SDLK_MINUS,  irr::EKEY_CODE::KEY_MINUS));
	KeyMap.push_back(SDLKeyMap(SDLK_PERIOD, irr::EKEY_CODE::KEY_PERIOD));

	// some special keys missing

	KeyMap.sort();
}

IrrlichtDevice* device;
irr::video::IVideoDriver * VideoDriver;
SDL_Window* rc_window;
irr::core::dimension2d<u32> rc_window_size;


//Canvases
struct rc_canvas_obj
{
    irr::video::ITexture* texture;

    irr::core::dimension2d<u32> dimension;

    struct rc_canvas_viewport
    {
        irr::core::vector2d<s32> position;
        irr::core::dimension2d<u32> dimension;
    } viewport;

    irr::core::vector2d<s32> offset;

    int mode;

    bool visible = true;
    int z = 0;

    irr::u8 alpha;

    irr::u32 color_mod;
};

irr::core::array<rc_canvas_obj> rc_canvas;
irr::core::array<u32> rc_canvas_zOrder;
int rc_active_canvas = -1;

irr::video::SColor rc_active_color(0,0,0,0);
irr::video::SColor rc_clear_color(0,0,0,0);

bool rc_init_events = false;
bool rc_init_timer = false;
bool rc_init_video = false;
bool rc_init_joystick = false;
bool rc_init_haptic = false;
bool rc_init_sensor = false;
bool rc_init_noparachute = false;
bool rc_init_audio = false;


irr::s32 MouseX, MouseY, MouseXRel, MouseYRel;
irr::u32 MouseButtonStates;


bool rcbasic_init()
{
    if(SDL_Init(SDL_INIT_EVENTS | SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC | SDL_INIT_SENSOR | SDL_INIT_NOPARACHUTE) < 0) //Audio causes init to fail on Fedora40 so I am leaving it out for now
    {
        bool rc_init_events = true;
        bool rc_init_timer = true;
        bool rc_init_video = true;
        bool rc_init_joystick = true;
        bool rc_init_haptic = true;
        bool rc_init_sensor = true;
        bool rc_init_noparachute = true;
        //os::Printer::log("SDL_Init Error: ", SDL_GetError());
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return false;
    }

    if(SDL_Init(SDL_INIT_AUDIO) < 0)
    {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        rc_init_audio = false;
    }

    device = NULL;
    VideoDriver = NULL;
    rc_window = NULL;

    return true;

}

bool rc_windowOpenEx(std::string title, int x, int y, int w, int h, uint32_t window_flags, irr::u8 AntiAlias, bool stencil_buffer, bool vsync)
{
    if(rc_window)
    {
        return false;
    }

    bool fullscreen = (window_flags & SDL_WINDOW_FULLSCREEN_DESKTOP) || (window_flags & SDL_WINDOW_FULLSCREEN);
    bool high_dpi = window_flags & SDL_WINDOW_ALLOW_HIGHDPI;
    bool borderless = window_flags & SDL_WINDOW_BORDERLESS;
    bool resizable = window_flags & SDL_WINDOW_RESIZABLE;
    bool visible = window_flags & SDL_WINDOW_SHOWN;

    uint32_t flags = SDL_WINDOW_OPENGL;
    flags |= (fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
    flags |= (high_dpi ? SDL_WINDOW_ALLOW_HIGHDPI : 0);
    flags |= (borderless ? SDL_WINDOW_BORDERLESS : 0);
    flags |= (resizable ? SDL_WINDOW_RESIZABLE : 0);
    flags |= (visible ? SDL_WINDOW_SHOWN : SDL_WINDOW_HIDDEN);

    //This size is used for virtual resolution
    rc_window_size.Width = w;
    rc_window_size.Height = h;

    rc_window = SDL_CreateWindow(title.c_str(), x, y, w, h, flags);

    //Get the actual size of the window to set the dimensions of the device
    if(rc_window)
        SDL_GetWindowSize(rc_window, &w, &h);

    SIrrlichtCreationParameters irr_creation_params;
    irr_creation_params.DeviceType = EIDT_SDL;
    irr_creation_params.DriverType = video::EDT_OPENGL;
    irr_creation_params.WindowId = rc_window;
    irr_creation_params.WindowSize = dimension2d<u32>((u32)w, (u32)h);
    irr_creation_params.Bits = 32;
    irr_creation_params.Fullscreen = fullscreen;
    irr_creation_params.Stencilbuffer = stencil_buffer;
    irr_creation_params.Vsync = vsync;
    irr_creation_params.EventReceiver = 0;
    irr_creation_params.WindowPosition = position2d<s32>(x, y);
    irr_creation_params.AntiAlias = AntiAlias;

	device = createDeviceEx(irr_creation_params);


	if (!device)
    {
        std::cout << "WindowOpen Error: Failed to Create Renderer" << std::endl;
        return false;
    }

    VideoDriver = device->getVideoDriver();

    return true;
}

bool rc_windowOpen(std::string title, int w, int h, bool fullscreen, bool vsync)
{
    uint32_t flags = SDL_WINDOW_SHOWN | (fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
    if(!rc_windowOpenEx(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w, h, flags, 0, true, vsync))
    {
        return false;
    }

    return true;
}

void rc_closeWindow_hw()
{
    if(rc_window!=NULL)
        SDL_DestroyWindow(rc_window);
    rc_window = NULL;
}

Uint32 rc_windowMode(int visible_flag, int fullscreen_flag, int resizable_flag, int borderless_flag, int highDPI_flag)
{
    Uint32 window_mode = ( visible_flag == 0 ? SDL_WINDOW_HIDDEN : SDL_WINDOW_SHOWN ) |
                         ( fullscreen_flag == 0 ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP ) |
                         ( resizable_flag == 0 ? 0 : SDL_WINDOW_RESIZABLE ) |
                         ( borderless_flag == 0 ? 0 : SDL_WINDOW_BORDERLESS ) |
                         ( highDPI_flag == 0 ? 0 : SDL_WINDOW_ALLOW_HIGHDPI );
    return window_mode;
}

Uint32 rc_getWindowMode()
{
    if(rc_window == NULL)
    {
        return 0;
    }
    return SDL_GetWindowFlags(rc_window);
}

void rc_raiseWindow()
{
    if(rc_window==NULL)
    {
        return;
    }
    SDL_RaiseWindow(rc_window);
}

void rc_showWindow()
{
    if(rc_window==NULL)
    {
        return;
    }
    SDL_ShowWindow(rc_window);
}

void rc_hideWindow()
{
    if(rc_window==NULL)
    {
        return;
    }
    SDL_HideWindow(rc_window);
}

void rc_getDesktopDisplayMode(int index, double * w, double * h, double * freq)
{
    SDL_DisplayMode dm;
    SDL_GetDesktopDisplayMode(index, &dm);
    *w = (double)dm.w;
    *h = (double)dm.h;
    *freq = (double)dm.refresh_rate;
}

void rc_setWindowTitle(std::string title)
{
    if(rc_window)
    {
        SDL_SetWindowTitle(rc_window, title.c_str());
    }
}

std::string rc_getWindowTitle()
{
    if(rc_window)
    {
        return SDL_GetWindowTitle(rc_window);
    }
    return "";
}

void rc_setWindowPosition(int x, int y)
{
    if(rc_window)
        SDL_SetWindowPosition(rc_window, x, y);
}

void rc_getWindowPosition(double * x, double * y)
{
    int x_data=0, y_data=0;
    if(rc_window)
        SDL_GetWindowPosition(rc_window,&x_data,&y_data);
    *x = x_data;
    *y = y_data;
}

void rc_setWindowSize(int w, int h)
{
    if(rc_window)
    {
        SDL_SetWindowSize(rc_window, w, h);

        irr::core::dimension2d<u32> win_size;
        int w, h;
        SDL_GetWindowSize(rc_window, &w, &h);
        win_size.Width = w;
        win_size.Height = h;

        device->setWindowSize(win_size);

        rc_window_size.Width = w;
        rc_window_size.Height = h;

    }
}

void rc_getWindowSize(double * w, double * h)
{
    int w_data = -1, h_data = -1;
    if(rc_window)
        SDL_GetWindowSize(rc_window, &w_data, &h_data);
    *w = w_data;
    *h = h_data;
}

void rc_setWindowMinSize(int w, int h)
{
    if(rc_window)
        SDL_SetWindowMinimumSize(rc_window, w, h);
}

void rc_getWindowMinSize(double * w, double * h)
{
    int w_data=0, h_data=0;
    if(rc_window)
        SDL_GetWindowMinimumSize(rc_window, &w_data, &h_data);
    *w = w_data;
    *h = h_data;
}

void rc_setWindowMaxSize(int w, int h)
{
    if(rc_window)
        SDL_SetWindowMaximumSize(rc_window, w, h);
}

void rc_getWindowMaxSize(double * w, double * h)
{
    int w_data=0, h_data=0;
    if(rc_window)
        SDL_GetWindowMaximumSize(rc_window, &w_data, &h_data);
    *w = w_data;
    *h = h_data;
}

bool rc_windowIsFullscreen()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        Uint32 wflags_cmp1 = wflags & SDL_WINDOW_FULLSCREEN;
        Uint32 wflags_cmp2 = wflags & SDL_WINDOW_FULLSCREEN_DESKTOP;

        if(wflags_cmp1 || wflags_cmp2)
            return true;
        else
            return false;
    }

    return false;

}

bool rc_windowIsVisible()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        if(wflags & SDL_WINDOW_SHOWN)
            return true;
        else
            return false;
    }

    return false;
}

bool rc_windowHasMouseFocus()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        if(wflags & SDL_WINDOW_MOUSE_FOCUS)
            return true;
        else
            return false;
    }

    return false;
}

bool rc_windowHasInputFocus()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        if(wflags & SDL_WINDOW_INPUT_FOCUS)
            return true;
        else
            return false;
    }

    return false;
}

bool rc_windowIsBordered()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        if(wflags & SDL_WINDOW_BORDERLESS)
            return false;
        else
            return true;
    }

    return false;
}

bool rc_windowIsResizable()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        if(wflags & SDL_WINDOW_RESIZABLE)
            return true;
        else
            return false;
    }

    return false;
}

bool rc_windowIsMinimized()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        if(wflags & SDL_WINDOW_MINIMIZED)
            return true;
        else
            return false;
    }

    return false;
}

bool rc_windowIsMaximized()
{
    if(rc_window)
    {
        Uint32 wflags = SDL_GetWindowFlags(rc_window);
        if(wflags & SDL_WINDOW_MAXIMIZED)
            return true;
        else
            return false;
    }

    return false;
}

bool rc_setWindowFullscreen(int flag)
{
    if(rc_window)
    {
        switch(flag)
        {
            case 0:
                flag = 0;
                break;
            default:
                flag = SDL_WINDOW_FULLSCREEN_DESKTOP;
                break;
        }

        Uint32 wflags_preOp = SDL_GetWindowFlags(rc_window);
        if( flag != 0 && ( (wflags_preOp & SDL_WINDOW_FULLSCREEN_DESKTOP) || (wflags_preOp & SDL_WINDOW_FULLSCREEN) ) )
            return true;
        else if( flag == 0 && !((wflags_preOp & SDL_WINDOW_FULLSCREEN_DESKTOP) || (wflags_preOp & SDL_WINDOW_FULLSCREEN)))
            return true;

        if(SDL_SetWindowFullscreen(rc_window, flag) < 0)
        {
            return false;
        }


        int w, h;
        SDL_GetWindowSize(rc_window, &w, &h);

        irr::core::dimension2d<u32> win_size(w, h);
        device->setWindowSize(win_size);

        if(!(w==rc_window_size.Width && h==rc_window_size.Height))
        {
            //TODO: change mouse scale adjust
        }

        SDL_PumpEvents();
        SDL_FlushEvents(SDL_FIRSTEVENT, SDL_LASTEVENT);

        return true;
    }

    return false;
}

bool rc_maximizeWindow()
{
    if(rc_window)
    {
        SDL_MaximizeWindow(rc_window);

        SDL_DisplayMode mode;
        SDL_GetWindowDisplayMode(rc_window, &mode);

        irr::core::dimension2d<u32> win_size(mode.w, mode.h);
        device->setWindowSize(win_size);

        return true;
    }

    return false;
}

bool rc_minimizeWindow()
{
    if(rc_window)
    {
        SDL_MinimizeWindow(rc_window);

        SDL_DisplayMode mode;
        SDL_GetWindowDisplayMode(rc_window, &mode);

        irr::core::dimension2d<u32> win_size(mode.w, mode.h);
        device->setWindowSize(win_size);

        return true;
    }

    return false;
}

void rc_setWindowBordered(bool b)
{
    SDL_bool bswitch = SDL_FALSE;
    if(b)
        bswitch = SDL_TRUE;
    if(rc_window)
        SDL_SetWindowBordered(rc_window, bswitch);
}

void rc_setWindowResizable(bool b)
{
    SDL_bool bswitch = SDL_FALSE;
    if(b)
        bswitch = SDL_TRUE;
    if(rc_window)
    {
        SDL_SetWindowResizable(rc_window, bswitch);
        device->setResizable(true);
    }
}

bool rc_media_windowExists()
{
    return (rc_window!=NULL);
}

bool rc_restoreWindow()
{
    if(rc_window)
    {
        SDL_RestoreWindow(rc_window);

        SDL_DisplayMode mode;
        SDL_GetWindowDisplayMode(rc_window, &mode);

        irr::core::dimension2d<u32> win_size(mode.w, mode.h);
        device->setWindowSize(win_size);

        return true;
    }

    return false;
}

void rc_setWindowIcon(int slot)
{
    SDL_Rect img_rect;
    img_rect.x = 0;
    img_rect.y = 0;
    //img_rect.w = rc_image_width[slot];
    //img_rect.h = rc_image_height[slot];
    /*
    if(rc_himage[slot][win_num] != NULL)
    {
        //SDL_RendererFlip rf = (SDL_RendererFlip)(SDL_FLIP_VERTICAL);

        SDL_Surface * tmp_surf = SDL_CreateRGBSurface(0, rc_image_width[slot], rc_image_height[slot], 32, 0, 0, 0, 0);
        SDL_Texture * tmp_tex = SDL_CreateTexture(rc_win_renderer[rc_active_window], rc_pformat->format, SDL_TEXTUREACCESS_TARGET, rc_image_width[slot], rc_image_height[slot]);
        SDL_SetRenderTarget(rc_win_renderer[rc_active_window],NULL);
        SDL_RenderCopy(rc_win_renderer[rc_active_window],rc_himage[slot][rc_active_window],NULL,&img_rect);
        //SDL_RenderCopyEx(rc_win_renderer[rc_active_window],rc_himage[slot][rc_active_window],NULL,NULL,0,NULL,rf);

        SDL_RenderReadPixels(rc_win_renderer[rc_active_window], &img_rect, rc_pformat->format,tmp_surf->pixels,tmp_surf->pitch);

        SDL_SetColorKey(tmp_surf,SDL_TRUE,rc_image_colorKey[slot]);

        SDL_SetWindowIcon(rc_win[rc_active_window], tmp_surf);


        if(rc_active_screen >= 0)
            SDL_SetRenderTarget(rc_win_renderer[rc_active_window], rc_hscreen[rc_active_window][rc_active_screen]);

        SDL_DestroyTexture(tmp_tex);
        SDL_FreeSurface(tmp_surf);
    }
    */
}

std::string rc_getClipboardText()
{
    return (std::string) SDL_GetClipboardText();
}

void rc_setClipboardText(std::string txt)
{
    SDL_SetClipboardText(txt.c_str());
}

int rc_hasClipboardText()
{
    return (int)SDL_HasClipboardText();
}




Uint32 rc_canvasOpen(int w, int h, int vx, int vy, int vw, int vh, int mode)
{
    if(!VideoDriver)
        return -1;

    rc_canvas_obj canvas;
    canvas.texture = VideoDriver->addRenderTargetTexture(irr::core::dimension2d<u32>(w,h), "rt");

    canvas.dimension.Width = w;
    canvas.dimension.Height = h;

    canvas.viewport.position.X = vx;
    canvas.viewport.position.Y = vy;
    canvas.viewport.dimension.Width = vw;
    canvas.viewport.dimension.Height = vh;

    canvas.offset.X = 0;
    canvas.offset.Y = 0;

    canvas.mode = mode;

    switch(mode)
    {
        case 0:
            break;
        case 1:
            VideoDriver->makeColorKeyTexture(canvas.texture, irr::video::SColor(0,0,0,0));
            break;
    }

    int canvas_id = -1;

    for(int i = 0; i < rc_canvas.size(); i++)
    {
        if(!rc_canvas[i].texture)
        {
            canvas_id = i;
            break;
        }
    }

    if(canvas_id < 0)
    {
        canvas_id = rc_canvas.size();
        rc_canvas.push_back(canvas);
    }

    if(rc_active_canvas < 0)
        rc_active_canvas = canvas_id;

    for(int i = 0; i < rc_canvas_zOrder.size(); i++)
    {
        if(rc_canvas_zOrder[i] == canvas_id)
        {
            rc_canvas_zOrder.erase(i);
            i--;
        }
    }

    rc_canvas_zOrder.push_back(canvas_id);


    return canvas_id;
}


void rc_canvasClose(int canvas_id)
{
    if(canvas_id < 0 || canvas_id >= rc_canvas.size())
        return;

    if(rc_canvas[canvas_id].texture != NULL)
        VideoDriver->removeTexture(rc_canvas[canvas_id].texture);

    rc_canvas[canvas_id].texture = NULL;

    if(rc_active_canvas == canvas_id)
        rc_active_canvas = -1;

    for(int i = 0; i < rc_canvas_zOrder.size(); i++)
    {
        if(rc_canvas_zOrder[i] == canvas_id)
        {
            rc_canvas_zOrder.erase(i);
            break;
        }
    }
}

void rc_setActiveCanvas(int canvas_id)
{
    rc_active_canvas = canvas_id;

    if(rc_active_canvas >= 0 && rc_active_canvas < rc_canvas.size())
    {
        if(rc_canvas[rc_active_canvas].texture)
            VideoDriver->setRenderTarget(rc_canvas[rc_active_canvas].texture);
    }
}

int rc_getActiveCanvas()
{
    return rc_active_canvas;
}

void rc_clearCanvas()
{
    if(rc_active_canvas >= 0 && rc_active_canvas < rc_canvas.size())
    {
        if(rc_canvas[rc_active_canvas].texture)
            VideoDriver->clearBuffers(true, true, true, rc_clear_color);
    }
}

void rc_setClearColor(Uint32 color)
{
    rc_clear_color.set(color);
}



Uint32 rc_rgba(Uint32 r, Uint32 g, Uint32 b, Uint32 a)
{
    irr::video::SColor color(a, r, g, b);
    return color.color;
}

Uint32 rc_rgb(Uint32 r, Uint32 g, Uint32 b)
{
    irr::video::SColor color(255, r, g, b);
    return color.color;
}


void rc_setColor(Uint32 color)
{
    rc_active_color.set(color);
}

void rc_drawRect(int x, int y, int w, int h)
{
    irr::core::vector2d<s32> r_pos(x,y);
    irr::core::dimension2d<s32> r_dim(w,h);
    irr::core::rect<s32> r(r_pos, r_dim);
    //std::cout << "drawRect: color=" << rc_active_color.color << " ( " << x << ", " << y << ", " << w << ", " << h << " ) " << std::endl;
    VideoDriver->draw2DRectangle(rc_active_color, r);
}




bool rc_update()
{
    if(!device->run())
        return false;

    int win_w = 0, win_h = 0;
    int w_scale = 1, h_scale = 1;

    if(rc_window)
    {
        SDL_GetWindowSize(rc_window, &win_w, &win_h);
        //std::cout << "size = " << win_w << ", " << win_h << std::endl;
    }

    SEvent irrevent;
	SDL_Event SDL_event;
	bool Close = false;

	while ( !Close && SDL_PollEvent( &SDL_event ) )
	{
		// os::Printer::log("event: ", core::stringc((int)SDL_event.type).c_str(),   ELL_INFORMATION);	// just for debugging

		switch ( SDL_event.type )
		{
		case SDL_MOUSEMOTION:
			irrevent.EventType = irr::EET_MOUSE_INPUT_EVENT;
			irrevent.MouseInput.Event = irr::EMIE_MOUSE_MOVED;
			MouseX = irrevent.MouseInput.X = SDL_event.motion.x;
			MouseY = irrevent.MouseInput.Y = SDL_event.motion.y;
			MouseXRel = SDL_event.motion.xrel;
			MouseYRel = SDL_event.motion.yrel;
			irrevent.MouseInput.ButtonStates = MouseButtonStates;

			device->postEventFromUser(irrevent);
			break;

		case SDL_MOUSEBUTTONDOWN:
		case SDL_MOUSEBUTTONUP:

			irrevent.EventType = irr::EET_MOUSE_INPUT_EVENT;
			irrevent.MouseInput.X = SDL_event.button.x;
			irrevent.MouseInput.Y = SDL_event.button.y;

			irrevent.MouseInput.Event = irr::EMIE_MOUSE_MOVED;

			switch(SDL_event.button.button)
			{
			case SDL_BUTTON_LEFT:
				if (SDL_event.type == SDL_MOUSEBUTTONDOWN)
				{
					irrevent.MouseInput.Event = irr::EMIE_LMOUSE_PRESSED_DOWN;
					MouseButtonStates |= irr::EMBSM_LEFT;
				}
				else
				{
					irrevent.MouseInput.Event = irr::EMIE_LMOUSE_LEFT_UP;
					MouseButtonStates &= !irr::EMBSM_LEFT;
				}

				//std::cout << "Position = " << SDL_event.button.x << ", " << SDL_event.button.y << std::endl;
				//rc_canvas[0].offset.X++;
				break;

			case SDL_BUTTON_RIGHT:
				if (SDL_event.type == SDL_MOUSEBUTTONDOWN)
				{
					irrevent.MouseInput.Event = irr::EMIE_RMOUSE_PRESSED_DOWN;
					MouseButtonStates |= irr::EMBSM_RIGHT;
				}
				else
				{
					irrevent.MouseInput.Event = irr::EMIE_RMOUSE_LEFT_UP;
					MouseButtonStates &= !irr::EMBSM_RIGHT;
				}
				//rc_canvas[0].offset.X--;
				break;

			case SDL_BUTTON_MIDDLE:
				if (SDL_event.type == SDL_MOUSEBUTTONDOWN)
				{
					irrevent.MouseInput.Event = irr::EMIE_MMOUSE_PRESSED_DOWN;
					MouseButtonStates |= irr::EMBSM_MIDDLE;
				}
				else
				{
					irrevent.MouseInput.Event = irr::EMIE_MMOUSE_LEFT_UP;
					MouseButtonStates &= !irr::EMBSM_MIDDLE;
				}
				break;

			}

			irrevent.MouseInput.ButtonStates = MouseButtonStates;

			if (irrevent.MouseInput.Event != irr::EMIE_MOUSE_MOVED)
			{
				device->postEventFromUser(irrevent);

				if ( irrevent.MouseInput.Event >= EMIE_LMOUSE_PRESSED_DOWN && irrevent.MouseInput.Event <= EMIE_MMOUSE_PRESSED_DOWN )
				{
					u32 clicks = device->checkSuccessiveClicks(irrevent.MouseInput.X, irrevent.MouseInput.Y, irrevent.MouseInput.Event);
					if ( clicks == 2 )
					{
						irrevent.MouseInput.Event = (EMOUSE_INPUT_EVENT)(EMIE_LMOUSE_DOUBLE_CLICK + irrevent.MouseInput.Event-EMIE_LMOUSE_PRESSED_DOWN);
						device->postEventFromUser(irrevent);
					}
					else if ( clicks == 3 )
					{
						irrevent.MouseInput.Event = (EMOUSE_INPUT_EVENT)(EMIE_LMOUSE_TRIPLE_CLICK + irrevent.MouseInput.Event-EMIE_LMOUSE_PRESSED_DOWN);
						device->postEventFromUser(irrevent);
					}
				}
			}
			break;

        case SDL_MOUSEWHEEL:
            irrevent.MouseInput.Event = irr::EMIE_MOUSE_WHEEL;
            irrevent.MouseInput.Wheel = SDL_event.wheel.y;
            break;

		case SDL_KEYDOWN:
		case SDL_KEYUP:
			{
				SDLKeyMap mp;
				mp.SDLKey = SDL_event.key.keysym.sym;
				s32 idx = KeyMap.binary_search(mp);

				EKEY_CODE key;
				if (idx == -1)
					key = (EKEY_CODE)0;
				else
					key = (EKEY_CODE)KeyMap[idx].Win32Key;

				irrevent.EventType = irr::EET_KEY_INPUT_EVENT;
				irrevent.KeyInput.Char = SDL_event.key.keysym.sym;
				irrevent.KeyInput.Key = key;
				irrevent.KeyInput.PressedDown = (SDL_event.type == SDL_KEYDOWN);
				irrevent.KeyInput.Shift = (SDL_event.key.keysym.mod & KMOD_SHIFT) != 0;
				irrevent.KeyInput.Control = (SDL_event.key.keysym.mod & KMOD_CTRL ) != 0;
				device->postEventFromUser(irrevent);
			}
			break;

		case SDL_QUIT:
			//std::cout << "CLOSE WIN" << std::endl;
			Close = true;
			break;

		case SDL_WINDOWEVENT:
			if (SDL_event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
			{
			// FIXME: Implement more precise window control
				// FIXME: Check if the window is game window
				s32 Width = SDL_event.window.data1;
                s32 Height = SDL_event.window.data2;
                //resizeWindow(Width, Height);
                if (VideoDriver)
                    VideoDriver->OnResize(core::dimension2d<u32>(Width, Height));

			}
			break;

		case SDL_USEREVENT:
			irrevent.EventType = irr::EET_USER_EVENT;
			irrevent.UserEvent.UserData1 = reinterpret_cast<uintptr_t>(SDL_event.user.data1);
			irrevent.UserEvent.UserData2 = reinterpret_cast<uintptr_t>(SDL_event.user.data2);

			device->postEventFromUser(irrevent);
			break;

		default:
			break;
		} // end switch

	} // end while

	if(!Close)
    {
        VideoDriver->setRenderTarget(0);
        VideoDriver->beginScene(true, true);

		for(int cz = 0; cz < rc_canvas_zOrder.size(); cz++)
        {
            int canvas_id = rc_canvas_zOrder[cz];

            if(rc_canvas[canvas_id].texture)
            {
                irr::core::rect<s32> dest(rc_canvas[canvas_id].viewport.position, rc_canvas[canvas_id].viewport.dimension);
                irr::core::rect<s32> src(rc_canvas[canvas_id].offset, rc_canvas[canvas_id].viewport.dimension);

                //std::cout << "draw canvas[" << canvas_id << "]" << std::endl;

                VideoDriver->draw2DImage(rc_canvas[canvas_id].texture, dest, src);
            }
        }

		//env->drawAll();

		VideoDriver->endScene();

		rc_setActiveCanvas(rc_active_canvas);
    }

    return (!Close);
}

#endif // RC_GFX_INCLUDED
