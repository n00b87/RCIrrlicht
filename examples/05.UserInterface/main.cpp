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


int main()
{

    rcbasic_init();

    rc_windowOpen("testing", 640, 480, false, true);


	SDL_Event event;
	bool quit = false;

	std::cout << "test start" << std::endl;

	uint32_t canvas1 = rc_canvasOpen(1000, 1000, 50, 50, 300, 300, 0);
	uint32_t canvas2 = rc_canvasOpen(1000, 1000, 100, 100, 300, 300, 0);
	rc_setCanvasVisible(canvas2, false);

	uint32_t canvas3 = rc_canvasOpen(1000, 1000, 150, 150, 300, 200, 0);

	rc_setCanvas3D(canvas3, true);

	//rc_setClearColor( rc_rgb(100, 100, 100));

	rc_setActiveCanvas(canvas1);
	rc_setClearColor(0);
	rc_clearCanvas();

	rc_setActiveCanvas(canvas2);
	rc_setClearColor( rc_rgb(0, 100, 0));
	rc_clearCanvas();

	rc_setActiveCanvas(canvas3);
	rc_setClearColor( rc_rgb(0, 0, 100));
	rc_setCameraPosition(150,-50,0);

	double rx = 0;
	double ry = 0;
	double rz = 0;
	//rc_setCameraRotation(rx, 0, 0);
	rc_clearCanvas();

	rc_setActiveCanvas(canvas1);

    rc_drawRect(50, 50, 90, 50);


    //tst(driver);

    rc_loadFont("NotoSansJP-VariableFont_wght.ttf", 12);

    int img = rc_loadImage("rcbasic.png");

    std::cout << "create sprite" << std::endl;
    int test_sprite = rc_createSprite(img);
    std::cout << "sprite has been created" << std::endl;


    rc_setSpriteBodyType(test_sprite, (int)b2_dynamicBody);
    //rc_setSpritePosition(test_sprite, 50, 30);


    int x = 20;
    int y = 20;
    int r = 0;

    int alpha = 255;

    double buf[] = { rc_rgb(255,0,0), rc_rgb(255,0,0), rc_rgb(255,0,0),
                     rc_rgb(0,255,0), rc_rgb(0,255,0), rc_rgb(0,255,0),
                     rc_rgb(0,0,255), rc_rgb(0,0,255), rc_rgb(0,0,255) };

    int b_img = rc_createImage(3, 3, buf);

    double b_out[9];

    rc_getImageBuffer(b_img, b_out);

    std::cout << "Color Red = " << rc_rgb(255,0,0) << std::endl;
    std::cout << "Color Green = " << rc_rgb(0,255,0) << std::endl;
    std::cout << "Color Blue = " << rc_rgb(0,0,255) << std::endl;

    for(int i = 0; i < 9; i++)
        std::cout << (Uint32)b_out[i] << std::endl;

    rc_setColorKey(b_img, rc_rgb(0,255,0));

    int img2 = rc_copyImage(img);

    //rc_setBilinearFilter(true);

    double w, h;

    rc_getImageSize(img2, &w, &h);

    std::cout << "image_size = " << w << ", " << h << std::endl;

    //rc_setColorMod(img2, rc_rgb(255,255,255));
    //rc_setBlendMode((int)irr::video::EBO_MIN_FACTOR);

    double zx = 1.0, zy = 1.0;

    bool h_flag = false;
    bool v_flag = false;

    int wclip = -1;

    int cx = 0, cy = 0;

    //int a = 255;

    int mesh1 = rc_loadMesh("../../media/sydney.md2");

    int actor1 = rc_createMeshActor(mesh1);
    int actor1_texture = rc_loadImage("../../media/sydney.bmp");
    rc_setActorTexture(actor1, 0, actor1_texture);
    rc_setActorMaterialFlag(actor1, EMF_LIGHTING, false);

    double cam_rot_x = 0;
    double cam_rot_y = 0;
    double cam_rot_z = 0;

    //std::cout << "Cam Rot = " << rc_canvas[canvas3].camera->getRotation().X << ", " << rc_canvas[canvas3].camera->getRotation().Y << ", " << rc_canvas[canvas3].camera->getRotation().Z << std::endl;

    device->getFileSystem()->addFileArchive("../../media/map-20kdm2.pk3");

	irr::scene::IAnimatedMesh *map = SceneManager->getMesh("20kdm2.bsp");
	device->getFileSystem()->removeFileArchive((irr::u32) 0);
	if (map)
	{
		irr::scene::ISceneNode *map_node = SceneManager->addOctreeSceneNode(map->getMesh(0));
		//Set position
		map_node->setPosition(vector3df(-850,-220,-850));
	}

    rc_setActiveCanvas(canvas3);

    Camera tst_cam;
    tst_cam.init(SceneManager, 0, 0, 0);

    double rot_y = -1;


	while(rc_update())
	{
	    rc_setClearColor(0);
	    rc_clearCanvas();
        bool get_color = false;

		rc_setActiveCanvas(canvas3);
		vector3df p0(0, 0, 0);
				vector3df p1(10, 30, 0);
				vector3df p2(20, -30, 0);
				vector3df p3(30, 0, 0);
				//drawBezierCurve(VideoDriver, p0, p1, p2, p3, irr::video::SColor(255, 0, 255, 0), 100);
		rc_setActiveCanvas(canvas1);


        if(rc_key(SDLK_w))
        {
            //y-= 3;
            rc_setActiveCanvas(canvas3);
            rc_translateCamera(0,0,10);
            rc_setActiveCanvas(canvas1);
        }
        else if(rc_key(SDLK_s))
        {
            //y+=3;
            rc_setActiveCanvas(canvas3);
            rc_translateCamera(0,0,-10);
            rc_setActiveCanvas(canvas1);
        }

        if(rc_key(SDLK_a))
        {
            //x-= 3;
            rc_setActiveCanvas(canvas3);
            rc_translateCamera(-10,0,0);
            rc_setActiveCanvas(canvas1);
        }
        else if(rc_key(SDLK_d))
        {
            //x+=3;
            rc_setActiveCanvas(canvas3);
            rc_translateCamera(10,0,0);
            rc_setActiveCanvas(canvas1);
        }



        if(rc_key(SDLK_1))
        {
            alpha--;
            if(alpha < 0)
                alpha = 0;
        }
        else if(rc_key(SDLK_2))
        {
            alpha++;
            if(alpha > 255)
                alpha = 255;
        }

        if(rc_key(SDLK_UP))
        {
            rc_setActiveCanvas(canvas3);

            cam_rot_x++;

            rc_rotateCamera(1, 0, 0);
            //rc_setCameraRotation(cam_rot_x, cam_rot_y, cam_rot_z);

            double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);

            rc_setActiveCanvas(canvas1);
        }
        else if(rc_key(SDLK_DOWN))
        {
            rc_setActiveCanvas(canvas3);

            cam_rot_x--;

            rc_rotateCamera(-1, 0, 0);
            //rc_setCameraRotation(cam_rot_x, cam_rot_y, cam_rot_z);

            double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);

            rc_setActiveCanvas(canvas1);
        }

        if(rc_key(SDLK_7))
        {
            rc_setActiveCanvas(canvas3);

            cam_rot_y--;

            //---test----------
            double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);

            rc_setCameraRotation(0, 244, 0);
            //rc_rotateCamera(0, cry, 0);
            //rc_rotateCamera(0, -1, 0);
            //rc_rotateCamera(crx, 0, 0);


			//rc_rotateCamera(-1*cam_rot_x, 0, 0);
			//rc_rotateCamera(0, -1, 0);
			//rc_rotateCamera(cam_rot_x, 0, 0);

            //---end test------

            //rc_rotateCamera(0, -1, 0);
            //rc_setCameraRotation(cam_rot_x, cam_rot_y, cam_rot_z);

            //double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);
            std::cout << "ROTY = " << cry << ", " << crz << std::endl;

            rc_setActiveCanvas(canvas1);
        }

        if(rc_key(SDLK_LEFT))
        {
            rc_setActiveCanvas(canvas3);

            cam_rot_y--;

            //---test----------
            double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);

			rc_rotateCamera(-1*cam_rot_x, 0, 0);
			rc_rotateCamera(0, -1, 0);
			rc_rotateCamera(cam_rot_x, 0, 0);

            //---end test------

            //rc_rotateCamera(0, -1, 0);
            //rc_setCameraRotation(cam_rot_x, cam_rot_y, cam_rot_z);

            //double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);
            std::cout << "ROTY = " << cry << ", " << crz << std::endl;

            rc_setActiveCanvas(canvas1);
        }
        else if(rc_key(SDLK_RIGHT))
        {
            rc_setActiveCanvas(canvas3);

            cam_rot_y++;

            rc_rotateCamera(0, 1, 0);
            //rc_setCameraRotation(cam_rot_x, cam_rot_y, cam_rot_z);

            double crx, cry, crz;
            rc_getCameraRotation(&crx, &cry, &crz);
            std::cout << "ROTY = " << cry << ", " << crz << std::endl;

            rc_setActiveCanvas(canvas1);
        }




        if(rc_key(SDLK_t))
        {
            rc_setCanvasZ(canvas1, rc_canvasZ(canvas1)+1);
        }
        else if(rc_key(SDLK_y))
        {
            rc_setCanvasZ(canvas2, rc_canvasZ(canvas2)+1);
        }
        else if(rc_key(SDLK_u))
        {
            rc_setCanvasZ(canvas3, rc_canvasZ(canvas3)+1);
        }

        if(rc_key(SDLK_i))
            y--;
        else if(rc_key(SDLK_k))
            y++;

        if(rc_key(SDLK_l))
            x++;
        else if(rc_key(SDLK_j))
            x--;

        //rc_setCanvasAlpha(canvas1, alpha);
        rc_setCanvasOffset(canvas1, 0, 0);
        //rc_setImageAlpha(img2, alpha);
        //rc_drawImage(img2, x, y);
        //rc_drawImage_flip(img2, x, y, h_flag, v_flag);
        //rc_drawImage_flipEx(img2, x, y, 47, 47, 48, 48, h_flag, v_flag);
        //rc_drawImage_zoom(img2, x, y, zx, zy);
        //rc_drawImage_zoomEx(img2, x, y, 20, 47, 62, 48, zx, zy);
        //rc_drawImage_rotozoomEx(img2, x, y, 20, 47, 62, 48, r, zx, zy);
        //rc_drawImage_rotateEx(img2, x, y, 30, 47, 62, 48, r);
        //rc_drawImage_blit(img, x, y, 0, 47, 48, 48);
        //rc_drawImage_blitEx(img, x, y, 96, 96, 0, 47, 48, 48);
        rc_setColor(rc_rgb(255,0,0));
        rc_drawRectFill(10,10,10,10);

        if(rc_key(SDLK_9))
		{
			rc_setColor(rc_rgb(0,255,0));
			rc_floodFill(15,15);
			rc_update();
			rc_wait(500);
			rc_waitKey();
		}

        //rc_drawImage_blitEx(b_img, 0,0, 30, 30, 0, 0, 3, 3);

        if(rc_key(SDLK_ESCAPE))
            break;

        if(get_color)
        {
            rc_setColor(rc_rgb(0,255,0));
            //rc_getPixel(rc_mouseX(),rc_mouseY());
            rc_setColor(rc_rgb(255,255,255));
        }
	}

	std::cout << "test end" << std::endl;

	SDL_DestroyWindow(rc_window);
	SDL_Quit();
	device->drop();

	return 0;
}

/*
**/
