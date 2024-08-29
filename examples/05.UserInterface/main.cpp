/** Example 005 User Interface

This tutorial shows how to use the built in User Interface of
the Irrlicht Engine. It will give a brief overview and show
how to create and use windows, buttons, scroll bars, static
texts, and list boxes.

As always, we include the header files, and use the irrlicht
namespaces. We also store a pointer to the Irrlicht device,
a counter variable for changing the creation position of a window,
and a pointer to a listbox.
*/
#include <irrlicht.h>
#include <vector>
#include <SDL2/SDL.h>

#include <codecvt>
#include <strstream>

#include "driverChoice.h"
#include "exampleHelper.h"

#include "gui_freetype_font.h"
#include "rc_stdlib.h"
#include "rc_gfx.h"
#include "rc_gfx3D.h"
#include "camera.h"

using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#ifdef _MSC_VER
#pragma comment(lib, "Irrlicht.lib")
#endif

void drawDebugInfo(int overlay_canvas, int view_canvas)
{
	rc_setActiveCanvas(view_canvas);
	double cam_x, cam_y, cam_z;
	rc_getCameraPosition(&cam_x, &cam_y, &cam_z);
	std::string cam_pos = rc_intern_str(cam_x) + ", " + rc_intern_str(cam_y) + ", " + rc_intern_str(cam_z);

	double rot_x, rot_y, rot_z;
	rc_getCameraRotation(&rot_x, &rot_y, &rot_z);
	std::string cam_rot = rc_intern_str(rot_x) + ", " + rc_intern_str(rot_y) + ", " + rc_intern_str(rot_z);

	rc_setActiveCanvas(overlay_canvas);
	rc_clearCanvas();
	rc_setColor(rc_rgb(255,255,255));

	rc_drawText("Position: " + cam_pos, 10, 10);
	rc_drawText("Rotation: " + cam_rot, 10, 30);
}

int main()
{

    rcbasic_init();

    rc_windowOpen("testing", 640, 480, false, true);


	SDL_Event event;
	bool quit = false;

	std::cout << "test start" << std::endl;

	uint32_t canvas1 = rc_canvasOpen(640, 480, 0, 0, 640, 480, 0);
	uint32_t canvas2 = rc_canvasOpen(640, 480, 0, 0, 640, 480, 0);

	rc_setCanvasZ(canvas2, 0);

	rc_setCanvas3D(canvas1, true);

    rc_loadFont("NotoSansJP-VariableFont_wght.ttf", 12);


    int mesh1 = rc_loadMesh("../../media/sydney.md2");

    int actor1 = rc_createMeshActor(mesh1);
    int actor1_texture = rc_loadImage("../../media/sydney.bmp");
    rc_setActorTexture(actor1, 0, actor1_texture);
    rc_setActorMaterialFlag(actor1, EMF_LIGHTING, false);

    //rc_setActorSolid(actor1, true);
    rc_setActorCollisionShape(actor1, RC_NODE_SHAPE_TYPE_CAPSULE, 1);
    rc_translateActor(actor1, 0, 150, 0);

	int level = rc_loadMeshFromArchive("../../media/map-20kdm2.pk3", "20kdm2.bsp");
	int actor2 = -1;

	if (level >= 0)
	{
		actor2 = rc_createMeshOctreeActor(level);
		rc_setActorSolid(actor2, true);
		rc_setActorCollisionShape(actor2, RC_NODE_SHAPE_TYPE_TRIMESH, 0);
	}

	while(rc_update())
	{
		if(rc_key(SDLK_ESCAPE))
			break;

		if(rc_key(SDLK_w))
        {
            rc_setActiveCanvas(canvas1);
            rc_translateCamera(0,0,10);
        }
        else if(rc_key(SDLK_s))
        {
            rc_setActiveCanvas(canvas1);
            rc_translateCamera(0,0,-10);
        }

        if(rc_key(SDLK_a))
        {
            rc_setActiveCanvas(canvas1);
            rc_translateCamera(-10,0,0);
        }
        else if(rc_key(SDLK_d))
        {
            rc_setActiveCanvas(canvas1);
            rc_translateCamera(10,0,0);
        }

        if(rc_key(SDLK_r))
		{
			rc_setActiveCanvas(canvas1);
			rc_translateCameraW(0, 10, 0);
		}
		else if(rc_key(SDLK_f))
		{
			rc_setActiveCanvas(canvas1);
			rc_translateCameraW(0, -10, 0);
		}


        if(rc_key(SDLK_UP))
        {
            rc_setActiveCanvas(canvas1);
            rc_rotateCamera(1, 0, 0);
        }
        else if(rc_key(SDLK_DOWN))
        {
            rc_setActiveCanvas(canvas1);
			rc_rotateCamera(-1, 0, 0);
        }

        if(rc_key(SDLK_LEFT))
        {
            rc_setActiveCanvas(canvas1);

            double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);

			rc_rotateCamera(-1*crx, 0, 0);
			rc_rotateCamera(0, -1, 0);
			rc_rotateCamera(crx, 0, 0);
        }
        else if(rc_key(SDLK_RIGHT))
        {
            rc_setActiveCanvas(canvas1);

            double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);

			rc_rotateCamera(-1*crx, 0, 0);
			rc_rotateCamera(0, 1, 0);
			rc_rotateCamera(crx, 0, 0);
        }


        drawDebugInfo(canvas2, canvas1);

	}

	std::cout << "test end" << std::endl;

	SDL_DestroyWindow(rc_window);
	SDL_Quit();
	device->drop();

	return 0;
}

/*
**/
