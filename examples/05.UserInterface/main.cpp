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
#include "camera.h"
#include "an8parser.h"
#include "rc_defines.h"
#include "rc_stdlib.h"
#include "rc_gfx.h"
#include "rc_gfx3D.h"
#include "rc_matrix.h"
#include "rc_geometry.h"
#include "rc_audio.h"
#include "rc_net.h"
#include "rc_video.h"

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

	double mouse_x, mouse_y, mb1, mb2, mb3;
	rc_getMouse(&mouse_x, &mouse_y, &mb1, &mb2, &mb3);
	std::string mouse_info = "";
	mouse_info += rc_intern_str(mouse_x) + ", ";
	mouse_info += rc_intern_str(mouse_y) + "   ";
	mouse_info += "LEFT = " + rc_intern_str(mb1) + ", ";
	mouse_info += "MIDDLE = " + rc_intern_str(mb2) + ", ";
	mouse_info += "RIGHT = " + rc_intern_str(mb3) + ", ";

	rc_setActiveCanvas(overlay_canvas);
	rc_clearCanvas();
	rc_setColor(rc_rgb(255,255,255));

	rc_drawText("Position: " + cam_pos, 10, 10);
	rc_drawText("Rotation: " + cam_rot, 10, 30);
	rc_drawText("Mouse Info: " + mouse_info, 10, 50);
}

void test_matrix1()
{
	irr::core::matrix4 m;
	m.setTranslation(irr::core::vector3df(44, 55, 66));
	m.setRotationDegrees(irr::core::vector3df(20, 70, 30));

	std::cout << "rot euler = " << m.getTranslation().X << ", " << m.getTranslation().Y << ", " << m.getTranslation().Z << std::endl;

	std::cout << std::endl << "debug output 1" << std::endl;
	printIrrMatrix(m);

	irr::core::vector3df rot = m.getRotationDegrees();

	std::cout << std::endl;

	std::cout << "ROT = " << rot.X << ", " << rot.Y << ", " << rot.Z << std::endl;

	irr::core::matrix4 m2;
	m2.setRotationDegrees(irr::core::vector3df(11, 12, 14));
	m2.setTranslation(irr::core::vector3df(23, 22, 19));
	m2.setScale(irr::core::vector3df(2,3,4));

	std::cout << std::endl;

	std::cout << std::endl << "debug output 2" << std::endl;
	printIrrMatrix(m2);

	m = m * m2;

	std::cout << std::endl;

	std::cout << std::endl << "debug output 3" << std::endl;
	printIrrMatrix(m);

	std::cout << std::endl;

	//printMatrix(m);
}

void test_matrix2()
{
	double x;
	double y;
	double z;

	int m = DimMatrix(NEW_MATRIX, 4, 4);
	rc_setIdentityMatrix(m, 4);
	rc_setMatrixTranslation(m, 44, 55, 66);
	rc_setMatrixRotation(m, 20, 70, 30);

	rc_getMatrixTranslation(m, &x, &y, &z);

	std::cout << "rot euler = " << x << ", " << y << ", " << z << std::endl;

	std::cout << std::endl << "debug output 1" << std::endl;
	printRCMatrix(m);

	rc_getMatrixRotation(m, &x, &y, &z);

	std::cout << std::endl;

	std::cout << "!!ROT = " << x << ", " << y << ", " << z << std::endl;

	int m2 = DimMatrix(NEW_MATRIX, 4, 4);
	rc_setIdentityMatrix(m2, 4);
	rc_setMatrixRotation(m2, 11, 12, 14);
	rc_setMatrixTranslation(m2, 23, 22, 19);
	rc_setMatrixScale(m2, 2,3,4);

	std::cout << std::endl;

	std::cout << std::endl << "debug output 2" << std::endl;
	printRCMatrix(m2);

	int mC = DimMatrix(NEW_MATRIX, 4, 4);

	rc_multiplyMatrix(m, m2, mC);

	std::cout << std::endl << "debug output 3" << std::endl;
	printRCMatrix(mC);

	std::cout << std::endl;

	//printMatrix(m);
}

void rcbasic_init()
{
	rc_audio_init();
    rc_gfx_init();
    rc_net_init();
}

int sprite_test()
{

    rcbasic_init();

    rc_windowOpen("testing", 640, 480, false, true);


	SDL_Event event;
	bool quit = false;

	std::cout << "test start" << std::endl;

	uint32_t canvas1 = rc_canvasOpenSpriteLayer(0, 0, 640, 480);
	uint32_t canvas2 = rc_canvasOpen(640, 480, 0, 0, 640, 480, 0);

	rc_setCanvasZ(canvas1, 0);
	rc_setCanvasZ(canvas2, 1);

    std::string fnt = "NotoSansJP-VariableFont_wght.ttf";
    rc_loadFont(fnt, 12);

    int img_a = rc_loadImage("graizor.png");
    int img_b = rc_loadImage("rcbasic.png");

    rc_setActiveCanvas(canvas1);
    int spriteA = rc_createSprite(img_a, 64, 64);
	int walk_animation_left = rc_createSpriteAnimation(spriteA, 4, 8);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 0, 28);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 1, 29);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 2, 30);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 3, 31);

	int walk_animation_right = rc_createSpriteAnimation(spriteA, 4, 8);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 0, 0);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 1, 1);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 2, 2);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 3, 3);

	rc_setSpriteAnimation(spriteA, walk_animation_right, -1);
	//rc_loopSpriteAnimation(spriteA, -1);

	int spriteB = rc_createSprite(img_b, 96, 96);
	rc_setSpritePosition(spriteB, 1, 100);

	//rc_setSpriteSolid(spriteA, true);
	//rc_setSpriteSolid(spriteB, true);

	rc_setSpriteZ(spriteA, 1);
	rc_setSpriteZ(spriteB, 0);

	while(rc_update())
	{
		if(rc_key(SDLK_ESCAPE))
			break;

		if(rc_key(SDLK_1))
		{
			rc_setSpriteSolid(spriteA, true);
			rc_setSpriteSolid(spriteB, true);
		}
		else if(rc_key(SDLK_2))
		{
			rc_setSpriteSolid(spriteA, false);
			rc_setSpriteSolid(spriteB, false);
		}
		else if(rc_key(SDLK_3))
		{
			std::cout << "3" << std::endl;
			rc_setSpriteAnimationLength(spriteA, walk_animation_left, 1);
		}
		else if(rc_key(SDLK_4))
		{
			std::cout << "4" << std::endl;
			rc_setSpriteFrame(spriteA, 21);
		}

		if(rc_key(SDLK_LEFT))
		{
			if(rc_getSpriteAnimation(spriteA)!=walk_animation_left)
				rc_setSpriteAnimation(spriteA, walk_animation_left, -1);
		}
		else if(rc_key(SDLK_RIGHT))
		{
			if(rc_getSpriteAnimation(spriteA)!=walk_animation_right)
				rc_setSpriteAnimation(spriteA, walk_animation_right, -1);
		}

		int lv = 30;
		if(rc_key(SDLK_a))
		{
			//rc_translateSprite(spriteB, -1, 0);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(-lv, 0));
		}
		else if(rc_key(SDLK_d))
		{
			//rc_translateSprite(spriteB, 1, 0);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(lv, 0));
		}

		if(rc_key(SDLK_w))
		{
			//rc_translateSprite(spriteB, 0, -1);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(0, -lv));
		}
		else if(rc_key(SDLK_s))
		{
			//rc_translateSprite(spriteB, 0, 1);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(0,lv));
		}

		//rc_rotateSprite(spriteB, 1);
		rc_sprite[spriteB].physics.body->SetAngularVelocity(10);
	}

	std::cout << "test end" << std::endl;

	SDL_DestroyWindow(rc_window);
	SDL_Quit();
	device->drop();

	return 0;
}


int tile_test()
{

    rcbasic_init();

    rc_windowOpen("testing", 640, 480, false, true);


	SDL_Event event;
	bool quit = false;

	std::cout << "test start" << std::endl;

	uint32_t canvas1 = rc_canvasOpenSpriteLayer(0, 0, 640, 480);
	uint32_t canvas2 = rc_canvasOpen(640, 480, 0, 0, 640, 480, 0);

	uint32_t canvas3 = rc_canvasOpen(640, 480, 0, 0, 640, 480, 0);

	rc_setCanvasZ(canvas1, 0);
	rc_setCanvasZ(canvas2, 1);
	rc_setCanvasZ(canvas3, 2);

	rc_setActiveCanvas(canvas3);
	rc_setColor(rc_rgb(120, 120, 120));
	rc_drawRectFill(0, 0, 640, 480);

    std::string fnt = "NotoSansJP-VariableFont_wght.ttf";
    rc_loadFont(fnt, 12);

    int img_a = rc_loadImage("graizor.png");
    int img_b = rc_loadImage("rcbasic.png");

    rc_setActiveCanvas(canvas1);
    int spriteA = rc_createSprite(img_a, 64, 64);
	int walk_animation_left = rc_createSpriteAnimation(spriteA, 4, 8);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 0, 28);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 1, 29);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 2, 30);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_left, 3, 31);

	int walk_animation_right = rc_createSpriteAnimation(spriteA, 4, 8);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 0, 0);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 1, 1);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 2, 2);
	rc_setSpriteAnimationFrame(spriteA, walk_animation_right, 3, 3);

	rc_setSpriteAnimation(spriteA, walk_animation_right, -1);
	//rc_loopSpriteAnimation(spriteA, -1);

	int spriteB = rc_createSprite(img_b, 96, 96);
	rc_setSpritePosition(spriteB, 1, 100);

	//rc_setSpriteSolid(spriteA, true);
	//rc_setSpriteSolid(spriteB, true);

	rc_setSpriteZ(spriteA, 1);
	rc_setSpriteZ(spriteB, 0);


	int tile_img = rc_loadImage("tiles2.png");

	std::cout << "img = " << tile_img << std::endl;

	rc_setActiveCanvas(canvas2);

	int tileset = rc_createTileSet(tile_img, 32, 32);
	int tilemap = rc_createTileMap(tileset, 500, 500);

	rc_fillTile(tilemap, 7, 3, 3, 4, 2);

	rc_setTileAnimationLength(tileset, 7, 2);
	rc_setTileAnimationSpeed(tileset, 7, 1);
	rc_setTileAnimationFrame(tileset, 7, 1, 8);

	int offset_x = 0;
	int offset_y = 0;


	while(rc_update())
	{
		if(rc_key(SDLK_ESCAPE))
			break;

		if(rc_key(SDLK_0))
		{
			std::cout << "Current Speed = " << rc_getTileAnimationSpeed(tileset, 7) << std::endl;
		}
		else if(rc_key(SDLK_1))
			rc_setTileAnimationSpeed(tileset, 7, rc_getTileAnimationSpeed(tileset, 7)+1);

		if(rc_key(SDLK_UP))
			offset_y -= 2;
		else if(rc_key(SDLK_DOWN))
			offset_y += 2;

		if(rc_key(SDLK_LEFT))
			offset_x -= 2;
		else if(rc_key(SDLK_RIGHT))
			offset_x += 2;

		if(rc_key(SDLK_1))
		{
			rc_setSpriteSolid(spriteA, true);
			rc_setSpriteSolid(spriteB, true);
		}
		else if(rc_key(SDLK_2))
		{
			rc_setSpriteSolid(spriteA, false);
			rc_setSpriteSolid(spriteB, false);
		}
		else if(rc_key(SDLK_3))
		{
			std::cout << "3" << std::endl;
			rc_setSpriteAnimationLength(spriteA, walk_animation_left, 1);
		}
		else if(rc_key(SDLK_4))
		{
			std::cout << "4" << std::endl;
			rc_setSpriteFrame(spriteA, 21);
		}

		if(rc_key(SDLK_a))
		{
			if(rc_getSpriteAnimation(spriteA)!=walk_animation_left)
				rc_setSpriteAnimation(spriteA, walk_animation_left, -1);
		}
		else if(rc_key(SDLK_d))
		{
			if(rc_getSpriteAnimation(spriteA)!=walk_animation_right)
				rc_setSpriteAnimation(spriteA, walk_animation_right, -1);
		}

		int lv = 30;
		if(rc_key(SDLK_a))
		{
			//rc_translateSprite(spriteB, -1, 0);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(-lv, 0));
		}
		else if(rc_key(SDLK_d))
		{
			//rc_translateSprite(spriteB, 1, 0);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(lv, 0));
		}

		if(rc_key(SDLK_w))
		{
			//rc_translateSprite(spriteB, 0, -1);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(0, -lv));
		}
		else if(rc_key(SDLK_s))
		{
			//rc_translateSprite(spriteB, 0, 1);
			rc_sprite[spriteA].physics.body->SetLinearVelocity(b2Vec2(0,lv));
		}

		//rc_rotateSprite(spriteB, 1);
		rc_sprite[spriteB].physics.body->SetAngularVelocity(10);

		rc_setCanvasOffset(canvas1, offset_x, offset_y);
		rc_drawTileMap(tilemap, 0, 0, 640, 480, offset_x, offset_y);
	}

	std::cout << "test end" << std::endl;

	SDL_DestroyWindow(rc_window);
	SDL_Quit();
	device->drop();

	return 0;
}


void control3D(double cam_speed)
{

	if(rc_key(SDLK_w))
		rc_translateCamera(0, 0, cam_speed);

	if(rc_key(SDLK_s))
		rc_translateCamera(0, 0, -cam_speed);

	if(rc_key(SDLK_a))
		rc_translateCamera(-cam_speed, 0, 0);

	if(rc_key(SDLK_d))
		rc_translateCamera(cam_speed, 0, 0);

	if(rc_key(SDLK_UP))
		rc_rotateCamera(cam_speed, 0, 0);

	if(rc_key(SDLK_DOWN))
		rc_rotateCamera(-cam_speed, 0, 0);

	if(rc_key(SDLK_LEFT))
	{
		double crx, cry, crz;
		rc_getCameraRotation(&crx, &cry, &crz);
		rc_setCameraRotation(crx, cry-cam_speed, crz);
		//rc_rotateCamera(0, -cam_speed, 0);
	}

	if(rc_key(SDLK_RIGHT))
	{
		double crx, cry, crz;
		rc_getCameraRotation(&crx, &cry, &crz);

		rc_setCameraRotation(crx, cry+cam_speed, crz);
		//rc_rotateCamera(0, cam_speed, 0);
	}
}

int actor_test()
{

    rcbasic_init();

    rc_windowOpen("testing", 640, 480, false, true);


	SDL_Event event;
	bool quit = false;

	std::cout << "test start" << std::endl;

	uint32_t canvas1 = rc_canvasOpen3D(0, 0, 640, 480, 0);
	uint32_t canvas2 = rc_canvasOpen(640, 480, 0, 0, 640, 480, 0);

	rc_setCanvasZ(canvas1, 0);
	rc_setCanvasZ(canvas2, 1);

    std::string fnt = "NotoSansJP-VariableFont_wght.ttf";
    rc_loadFont(fnt, 12);

    std::string media_path = "../../media/";
    int q3map_mesh = rc_loadMeshFromArchive(media_path + "map-20kdm2.pk3", "20kdm2.bsp");
    int q3map = rc_createOctreeActor(q3map_mesh);

    int tst_mesh = rc_loadMesh(media_path + "dwarf.x");
    int tst_actor = rc_createAnimatedActor(tst_mesh);

    int tst_texture = rc_loadImage(media_path + "dwarf.jpg");
    rc_setActorTexture(tst_actor, 0, tst_texture);
    int tst_material = rc_getActorMaterial(tst_actor, 0);
    rc_setMaterialLighting(tst_material, false);

    int a1 = rc_createActorAnimation(tst_actor, 0, 8, 24);
    //rc_setActorAnimation(tst_actor, a1, 0);

    rc_setActorPosition(q3map, -1350,-130,-1400);
    rc_setActorPosition(tst_actor, -90,-15,-140);

    rc_setActiveCanvas(canvas1);

	double cam_speed = 3;

	rc_rotateCamera(0, 180, 0);
	irr::scene::IAnimatedMeshSceneNode* node = (irr::scene::IAnimatedMeshSceneNode*)rc_actor[tst_actor].mesh_node;
	std::cout << "mesh type = " << ((int)node->getMesh()->getMeshType()) << std::endl;
	//std::cout << "mesh joint_used = " << ((int)node->) << std::endl;

	node->setDebugDataVisible(irr::scene::EDS_MESH_WIRE_OVERLAY);

	while(rc_update())
	{
		if(rc_key(SDLK_ESCAPE))
			break;

		if(rc_key(SDLK_SPACE))
			rc_setActorPosition(tst_actor, 0, 0, 0);

		if(rc_key(SDLK_1))
			rc_setActorAnimation(tst_actor, a1, 2);

		if(!rc_actorAnimationIsPlaying(tst_actor))
			rc_setActorFrame(tst_actor, 0);

		if(rc_key(SDLK_2) && (!rc_actorIsInTransition(tst_actor)))
		{
			rc_startActorTransition(tst_actor, 8, 10);
			//rc_setActorAnimationSpeed(tst_actor, a1, rc_getActorAnimationSpeed(tst_actor, a1)+1);
		}

		control3D(cam_speed);
	}

	std::cout << "test end" << std::endl;

	SDL_DestroyWindow(rc_window);
	SDL_Quit();
	device->drop();

	return 0;
}

int main()
{
	//actor_test();
	//sprite_test();
	tile_test();
	return 0;
}

/*
**/
