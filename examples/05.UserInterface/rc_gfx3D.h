#ifndef RC_GFX3D_H_INCLUDED
#define RC_GFX3D_H_INCLUDED

#include <SDL2/SDL.h>
#include <irrlicht.h>
#include <iostream>
#include <sstream>
#include <string>
#include <locale>
#include <codecvt>
#include <cmath>
#include <set>

#include "camera.h"
#include "rc_gfx_core.h"


//load a mesh from a file
int rc_loadMesh(std::string mesh_file)
{
    int mesh_id = -1;

    rc_mesh_obj mesh_obj;
    mesh_obj.mesh_type = RC_MESH_TYPE_ANIMATED;

    irr::scene::IAnimatedMesh* mesh = SceneManager->getMesh(mesh_file.c_str());
    mesh_obj.mesh = mesh;

    if(!mesh)
        return -1;

    for(int i = 0; i < rc_mesh.size(); i++)
    {
        if(!rc_mesh[i].mesh)
        {
            mesh_id = i;
            break;
        }
    }

    if(mesh_id < 0)
    {
        mesh_id = rc_mesh.size();
        rc_mesh.push_back(mesh_obj);
    }
    else
    {
        rc_mesh[mesh_id] = mesh_obj;
    }

    return mesh_id;
}

//delete mesh
void rc_deleteMesh(int mesh_id)
{
    if(mesh_id < 0 || mesh_id >= rc_mesh.size())
        return;

    if(rc_mesh[mesh_id].mesh)
        rc_mesh[mesh_id].mesh->drop();

    rc_mesh[mesh_id].mesh = NULL;
    rc_mesh[mesh_id].mesh_type = 0;

}

//create mesh from geometry data [TODO]
int rc_createMesh()
{
    irr::scene::ISkinnedMesh * mesh = SceneManager->createSkinnedMesh();

    if(!mesh)
        return -1;

    int mesh_id = rc_mesh.size();
    rc_mesh_obj mesh_obj;
    mesh_obj.mesh = mesh;
    mesh_obj.mesh_type = RC_MESH_TYPE_ANIMATED;

    rc_mesh.push_back(mesh_obj);

    return mesh_id;
}


//create mesh from geometry data [TODO]
bool rc_addMeshBuffer(int mesh_id, int vertex_count, double* vertex_data, double* normal_data, double* uv_data, int index_count, double* index_data)
{
    irr::scene::ISkinnedMesh * mesh = (irr::scene::ISkinnedMesh*) rc_mesh[mesh_id].mesh;

    irr::scene::SSkinMeshBuffer* mbuf = mesh->addMeshBuffer();

    if(!mbuf)
    {
        mesh->drop();
        return false;
    }

    irr::core::array<irr::video::S3DVertex> vertices;
    irr::core::array<irr::u16> indices;

    for(int i = 0; i < vertex_count; i++)
    {
        irr::video::S3DVertex v;
        v.Pos = irr::core::vector3df( (irr::f32) vertex_data[i*3], (irr::f32) vertex_data[i*3+1], (irr::f32) vertex_data[i*3+2] );
        v.Normal = irr::core::vector3df( (irr::f32) normal_data[i*3], (irr::f32) normal_data[i*3+1], (irr::f32) normal_data[i*3+2] );
        v.TCoords = irr::core::vector2df( (irr::f32) uv_data[i*2], (irr::f32) uv_data[i*2+1] );
        vertices.push_back(v);
    }

    for(int i = 0; i < index_count; i++)
    {
        indices.push_back( (irr::u16) index_data[i] );
    }

    if(indices.size() > 0)
    {
        for(int i = 0; i < vertices.size(); i++)
            mbuf->Vertices_Standard.push_back(vertices[i]);

        for(int i = 0; i < indices.size(); i++)
            mbuf->Indices.push_back(indices[i]);

    }

    return true;
}



//add mesh actor to scene
int rc_createMeshActor(int mesh_id)
{
    if(mesh_id < 0 || mesh_id >= rc_mesh.size())
        return -1;

    irr::scene::IAnimatedMesh* mesh = rc_mesh[mesh_id].mesh;

    if(!mesh)
        return -1;

    int actor_id = -1;
    irr::scene::IAnimatedMeshSceneNode* node = SceneManager->addAnimatedMeshSceneNode(mesh);
    rc_scene_node actor;
    actor.node_type = RC_NODE_TYPE_MESH;
    actor.mesh_node = node;

    if(!node)
        return -1;

    for(int i = 0; i < rc_actor.size(); i++)
    {
        if(!rc_actor[i].mesh_node)
        {
            actor_id = i;
            break;
        }
    }

    if(actor_id < 0)
    {
        actor_id = rc_actor.size();
        rc_actor.push_back(actor);
    }
    else
    {
        rc_actor[actor_id] = actor;
    }

    return actor_id;
}

//delete actor
void rc_deleteActor(int actor_id)
{
    if(actor_id < 0 || actor_id >= rc_actor.size())
        return;

    if(!rc_actor[actor_id].mesh_node)
        return;

    rc_actor[actor_id].mesh_node->remove();
    rc_actor[actor_id].mesh_node = NULL;
    rc_actor[actor_id].node_type = 0;
}

//set actor texture
void rc_setActorTexture(int actor, int layer, int image_id)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(!rc_actor[actor].mesh_node)
        return;

    if(image_id < 0 || image_id >= rc_image.size())
        return;

    if(rc_image[image_id].image)
    {
        rc_actor[actor].mesh_node->setMaterialTexture(layer, rc_image[image_id].image);
    }
}

//set actor texture
void rc_setActorTextureEx(int actor, int material, int layer, int resource_type, int resource_id)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(!rc_actor[actor].mesh_node)
        return;

    if(resource_type == RC_ACTOR_TEXTURE_TYPE_IMAGE)
    {
        int image_id = resource_id;

        if(image_id < 0 || image_id >= rc_image.size())
            return;

        if(rc_image[image_id].image)
        {
            rc_actor[actor].mesh_node->getMaterial(material).setTexture(layer, rc_image[image_id].image);
        }
    }
    else if(resource_type == RC_ACTOR_TEXTURE_TYPE_CANVAS)
    {
        int canvas_id = resource_id;

        if(canvas_id < 0 || canvas_id >= rc_canvas.size())
            return;

        if(rc_canvas[canvas_id].texture)
        {
            rc_actor[actor].mesh_node->getMaterial(material).setTexture(layer, rc_canvas[canvas_id].texture);
        }
    }
}

//get Material count
Uint32 rc_getActorMaterialCount(int actor)
{
    if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(!rc_actor[actor].mesh_node)
        return 0;

    return rc_actor[actor].mesh_node->getMaterialCount();
}

//set Actor Material Flag
void rc_setActorMaterialFlag(int actor, int flag, bool flag_value)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(!rc_actor[actor].mesh_node)
        return;

    rc_actor[actor].mesh_node->setMaterialFlag((irr::video::E_MATERIAL_FLAG)flag, flag_value);
}

//set Actor Material Flag
void rc_setActorMaterialFlagEx(int actor, int material, int flag, bool flag_value)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(!rc_actor[actor].mesh_node)
        return;

    rc_actor[actor].mesh_node->getMaterial(material).setFlag((irr::video::E_MATERIAL_FLAG)flag, flag_value);
}

//set Actor Material Flag
bool rc_getActorMaterialFlag(int actor, int material, int flag)
{
    if(actor < 0 || actor >= rc_actor.size())
        return false;

    if(!rc_actor[actor].mesh_node)
        return false;

    return rc_actor[actor].mesh_node->getMaterial(material).getFlag((irr::video::E_MATERIAL_FLAG)flag);
}

//Set Actor Material Type
void rc_setActorMaterialType(int actor, int material_type)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(!rc_actor[actor].mesh_node)
        return;

    irr::video::E_MATERIAL_TYPE n = (irr::video::E_MATERIAL_TYPE) material_type;
    rc_actor[actor].mesh_node->setMaterialType(n);
}

//Set Actor Material Type
void rc_setActorMaterialTypeEx(int actor, int material, int material_type)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(!rc_actor[actor].mesh_node)
        return;

    irr::video::E_MATERIAL_TYPE n = (irr::video::E_MATERIAL_TYPE) material_type;
    rc_actor[actor].mesh_node->getMaterial(material).MaterialType = n;
}

//Set Actor Material Type
int rc_getActorMaterialType(int actor, int material)
{
    if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(!rc_actor[actor].mesh_node)
        return 0;

    return (int)rc_actor[actor].mesh_node->getMaterial(material).MaterialType;
}

struct rc_material_obj
{
    irr::video::SMaterial mat;
    bool isUsed = false;
};

irr::core::array<rc_material_obj> rc_material;

//Set Actor Material Type
void rc_setActorMaterial(int actor, int material_num, int material_id)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(!rc_actor[actor].mesh_node)
        return;

    if(material_id < 0 || material_id >= rc_material.size())
        return;

    if(rc_material[material_id].isUsed)
        rc_actor[actor].mesh_node->getMaterial(material_num) = rc_material[material_id].mat;
}

//Set Actor Material Type
int rc_getActorMaterial(int actor, int material_num)
{
    if(actor < 0 || actor >= rc_actor.size())
        return -1;

    if(!rc_actor[actor].mesh_node)
        return -1;

    int material_id = -1;

    for(int i = 0; i < rc_material.size(); i++)
    {
        if(!rc_material[i].isUsed)
        {
            rc_material[i].isUsed = true;
            material_id = i;
            break;
        }
    }

    if(material_id < 0)
    {
        material_id = rc_material.size();
        rc_material_obj nmat;
        nmat.isUsed = true;
        nmat.mat = rc_actor[actor].mesh_node->getMaterial(material_num);
        rc_material.push_back(nmat);
    }
    else
        rc_material[material_id].mat = rc_actor[actor].mesh_node->getMaterial(material_num);

    return material_id;
}

//get Actor Texture
int rc_getActorTexture(int actor, int material, int layer)
{
    if(actor < 0 || actor >= rc_actor.size())
        return -1;

    if(!rc_actor[actor].mesh_node)
        return -1;

    rc_image_obj img;
    img.image = rc_actor[actor].mesh_node->getMaterial(material).getTexture(layer);

    if(img.image == NULL)
        return -1;

    int img_id = -1;

    for(int i = 0; i < rc_image.size(); i++)
    {
        if(rc_image[i].image == NULL)
        {
            img_id = i;
            break;
        }
    }

    if(img_id < 0)
    {
        img_id = rc_image.size();
        rc_image.push_back(img);
    }
    else
    {
        rc_image[img_id] = img;
    }

    return img_id;
}


//set actor position
void rc_setActorPosition(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    switch(rc_actor[actor].node_type)
    {
        case RC_NODE_TYPE_MESH:
            rc_actor[actor].mesh_node->setPosition( irr::core::vector3d<irr::f32>((irr::f32)x, (irr::f32)y, (irr::f32)z));
            break;
    }
}

//translate actor from local orientation
void rc_translateActor(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    translateNode(rc_actor[actor].mesh_node, irr::core::vector3df(x,y,z));
}

//get actor position
void rc_getActorPosition(int actor, double* x, double* y, double* z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = (double)rc_actor[actor].mesh_node->getAbsolutePosition().X;
    *y = (double)rc_actor[actor].mesh_node->getAbsolutePosition().Y;
    *z = (double)rc_actor[actor].mesh_node->getAbsolutePosition().Z;
}

//set actor scale
void rc_setActorScale(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    switch(rc_actor[actor].node_type)
    {
        case RC_NODE_TYPE_MESH:
            rc_actor[actor].mesh_node->setScale( irr::core::vector3d<irr::f32>((irr::f32)x, (irr::f32)y, (irr::f32)z));
            break;
    }
}

//scale actor
void rc_scaleActor(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    double sx = rc_actor[actor].mesh_node->getScale().X;
    double sy = rc_actor[actor].mesh_node->getScale().Y;
    double sz = rc_actor[actor].mesh_node->getScale().Z;
    rc_actor[actor].mesh_node->setScale( irr::core::vector3d<irr::f32>((irr::f32)sx*x, (irr::f32)sy*y, (irr::f32)sz*z));
}

//get actor scale
void rc_getActorScale(int actor, double* x, double* y, double* z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = rc_actor[actor].mesh_node->getScale().X;
    *y = rc_actor[actor].mesh_node->getScale().Y;
    *z = rc_actor[actor].mesh_node->getScale().Z;
}


//set actor rotation
void rc_setActorRotation(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    switch(rc_actor[actor].node_type)
    {
        case RC_NODE_TYPE_MESH:
            rc_actor[actor].mesh_node->setRotation( irr::core::vector3d<irr::f32>((irr::f32)x, (irr::f32)y, (irr::f32)z));
            break;
    }
}

//rotate actor
void rc_rotateActor(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    rotateNode(rc_actor[actor].mesh_node, irr::core::vector3df(x,y,z));
}

//get actor position
void rc_getActorRotation(int actor, double* x, double* y, double* z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = (double)rc_actor[actor].mesh_node->getRotation().X;
    *y = (double)rc_actor[actor].mesh_node->getRotation().Y;
    *z = (double)rc_actor[actor].mesh_node->getRotation().Z;
}

//set actor animation [TODO]
void rc_setActorAnimation(int actor, int start_frame, int end_frame)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    switch(rc_actor[actor].node_type)
    {
        case RC_NODE_TYPE_MESH:
            //rc_actor[actor].mesh_node->setAnimation();
            break;
    }
}

//set actor animation speed
void rc_setActorAnimationSpeed(int actor, double speed)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    switch(rc_actor[actor].node_type)
    {
        case RC_NODE_TYPE_MESH:
            rc_actor[actor].mesh_node->setAnimationSpeed( (irr::f32)speed );
            break;
    }
}


void rc_setCameraPosition(double x, double y, double z)
{
    if(!(rc_active_canvas > 0 && rc_active_canvas < rc_canvas.size()))
        return;

    rc_canvas[rc_active_canvas].camera.setPosition(x, y, z);
}

void rc_getCameraPosition(double* x, double* y, double* z)
{
    if(!(rc_active_canvas > 0 && rc_active_canvas < rc_canvas.size()))
        return;

    irr::f32 fx, fy, fz;

    rc_canvas[rc_active_canvas].camera.getPosition(fx, fy, fz);

    *x = fx;
    *y = fy;
    *z = fz;
}

void rc_translateCamera(double x, double y, double z)
{
    if(!(rc_active_canvas > 0 && rc_active_canvas < rc_canvas.size()))
        return;

    rc_canvas[rc_active_canvas].camera.translate(x, y, z);
}

void rc_setCameraRotation(double x, double y, double z)
{
    if(!(rc_active_canvas > 0 && rc_active_canvas < rc_canvas.size()))
        return;

    rc_canvas[rc_active_canvas].camera.setRotation(x, y, z);
}

void rc_getCameraRotation(double* x, double* y, double* z)
{
    if(!(rc_active_canvas > 0 && rc_active_canvas < rc_canvas.size()))
        return;

    *x = rc_canvas[rc_active_canvas].camera.rx;
    *y = rc_canvas[rc_active_canvas].camera.ry;
    *z = rc_canvas[rc_active_canvas].camera.rz;
}

void rc_rotateCamera(double x, double y, double z)
{
    if(!(rc_active_canvas > 0 && rc_active_canvas < rc_canvas.size()))
        return;

    rc_canvas[rc_active_canvas].camera.rotate(x, y, z);
}


#endif // RC_GFX3D_H_INCLUDED
