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
#include "rc_matrix.h"

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>

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

//load a mesh from an archive
int rc_loadMeshFromArchive(std::string archive, std::string mesh_file)
{
	int mesh_id = -1;

	device->getFileSystem()->addFileArchive(archive.c_str());
	irr::scene::IAnimatedMesh *mesh = SceneManager->getMesh(mesh_file.c_str());
	device->getFileSystem()->removeFileArchive((irr::u32) 0);

	rc_mesh_obj mesh_obj;
    mesh_obj.mesh_type = RC_MESH_TYPE_ANIMATED;
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

    //Actor RigidBody
    rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_NONE;
    //rc_actor[actor_id].physics.rigid_body = NULL;

    rc_actor[actor_id].physics.rigid_body = NULL;
	rc_actor[actor_id].physics.ghost = NULL;
    rc_actor[actor_id].physics.isSolid = false;

    rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_BOX;
	IBoxShape* shape = new IBoxShape(rc_actor[actor_id].mesh_node, 0, false);
	rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
	rc_actor[actor_id].physics.mass = 0;
	rc_actor[actor_id].physics.ghost->getIdentification()->setId(actor_id);

	//rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_SPHERE;
	//ISphereShape* shape = new ISphereShape(rc_actor[actor_id].mesh_node, 0, false);
	//rc_actor[actor_id].physics.shape_ref = shape;
	//rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
	//rc_actor[actor_id].physics.isSolid = true;

	//std::cout << "testing" << std::endl;

    //ghostObject = new btGhostObject();
	//ghostObject->setCollisionShape(new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.))));
	//ghostObject->setWorldTransform(groundTransform);
	//m_dynamicsWorld->addCollisionObject(ghostObject);
	//m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	//**SAMPLE CODE**/
	//btTransform trans=body->getCenterOfMassTransform();
	//btQuaternion quat;
	//quat.setEuler(irr::core::DEGTORAD*x,irr::core::DEGTORAD*y,irr::core::DEGTORAD*z);
	//trans.setRotation(quat);
	//body->setCenterOfMassTransform(trans);


    return actor_id;
}


//add mesh actor to scene
int rc_createMeshOctreeActor(int mesh_id)
{
    if(mesh_id < 0 || mesh_id >= rc_mesh.size())
        return -1;

    irr::scene::IAnimatedMesh* mesh = rc_mesh[mesh_id].mesh;

    if(!mesh)
        return -1;

    int actor_id = -1;
    //irr::scene::IAnimatedMeshSceneNode* node = SceneManager->addAnimatedMeshSceneNode(mesh);
    irr::scene::IOctreeSceneNode *node = SceneManager->addOctreeSceneNode(mesh);
    rc_scene_node actor;
    actor.node_type = RC_NODE_TYPE_OTMESH;
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

    //Actor RigidBody
    //rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_NONE;
    //rc_actor[actor_id].physics.rigid_body = NULL;

    rc_actor[actor_id].physics.rigid_body = NULL;
	rc_actor[actor_id].physics.ghost = NULL;

    rc_actor[actor_id].physics.isSolid = false;
    //rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_BOX;
	//IBoxShape* shape = new IBoxShape(rc_actor[actor_id].mesh_node, 0, false);
	//rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
	rc_actor[actor_id].physics.mass = 0;
	//rc_actor[actor_id].physics.ghost->getIdentification()->setId(actor_id);


	rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_SPHERE;
	ISphereShape* shape = new ISphereShape(rc_actor[actor_id].mesh_node, 0, false);
	rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
	rc_actor[actor_id].physics.rigid_body->includeNodeOnRemoval(false);
	rc_actor[actor_id].physics.rigid_body->getIdentification()->setId(actor_id);
	rc_actor[actor_id].physics.rigid_body->getPointer()->setActivationState(ACTIVE_TAG);
	rc_actor[actor_id].physics.rigid_body->getPointer()->setActivationState(DISABLE_DEACTIVATION);

    //ghostObject = new btGhostObject();
	//ghostObject->setCollisionShape(new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.))));
	//ghostObject->setWorldTransform(groundTransform);
	//m_dynamicsWorld->addCollisionObject(ghostObject);
	//m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	//**SAMPLE CODE**/
	//btTransform trans=body->getCenterOfMassTransform();
	//btQuaternion quat;
	//quat.setEuler(irr::core::DEGTORAD*x,irr::core::DEGTORAD*y,irr::core::DEGTORAD*z);
	//trans.setRotation(quat);
	//body->setCenterOfMassTransform(trans);


    return actor_id;
}


//Set Gravity
void rc_setGravity3D(double x, double y, double z)
{
	rc_physics3D.world->setGravity(irr::core::vector3d<f32>(x, y, z));
}

void rc_setActorCollisionShape(int actor_id, int shape_type, double mass)
{
	if(rc_actor[actor_id].physics.rigid_body)
	{
		rc_physics3D.world->removeCollisionObject(rc_actor[actor_id].physics.rigid_body, false);
		delete rc_actor[actor_id].physics.rigid_body;
	}

	if(rc_actor[actor_id].physics.ghost)
	{
		rc_physics3D.world->removeCollisionObject(rc_actor[actor_id].physics.ghost, false);
		delete rc_actor[actor_id].physics.ghost;
	}

	rc_actor[actor_id].physics.rigid_body = NULL;
	rc_actor[actor_id].physics.ghost = NULL;

	if(rc_actor[actor_id].physics.isSolid)
		rc_actor[actor_id].physics.mass = mass;
	else
		rc_actor[actor_id].physics.mass = 0;

	switch(shape_type)
	{
		case RC_NODE_SHAPE_TYPE_NONE:
			break;

		case RC_NODE_SHAPE_TYPE_BOX:
			{
				rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_BOX;
				IBoxShape* shape = new IBoxShape(rc_actor[actor_id].mesh_node, mass, false);

				if(rc_actor[actor_id].physics.isSolid)
					rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
				else
					rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
			}
			break;

		case RC_NODE_SHAPE_TYPE_SPHERE:
			{
				rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_SPHERE;
				ISphereShape* shape = new ISphereShape(rc_actor[actor_id].mesh_node, mass, false);

				if(rc_actor[actor_id].physics.isSolid)
					rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
				else
					rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
			}
			break;

		case RC_NODE_SHAPE_TYPE_CYLINDER:
			{
				rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_CYLINDER;
				ICylinderShape* shape = new ICylinderShape(rc_actor[actor_id].mesh_node, mass, false);

				if(rc_actor[actor_id].physics.isSolid)
					rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
				else
					rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
			}
			break;

		case RC_NODE_SHAPE_TYPE_CAPSULE:
			{
				rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_CAPSULE;
				ICapsuleShape* shape;

				if(rc_actor[actor_id].node_type == RC_NODE_TYPE_MESH)
				{
					irr::scene::IAnimatedMeshSceneNode* node = (irr::scene::IAnimatedMeshSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new ICapsuleShape(node, mass, false);
				}
				else if(rc_actor[actor_id].node_type == RC_NODE_TYPE_OTMESH)
				{
					irr::scene::IOctreeSceneNode* node = (irr::scene::IOctreeSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new ICapsuleShape(node, mass, false);
				}
				else if(rc_actor[actor_id].node_type == RC_NODE_TYPE_TERRAIN)
				{
					irr::scene::ITerrainSceneNode* node = (irr::scene::ITerrainSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new ICapsuleShape(node, mass, false);
				}

				if(rc_actor[actor_id].physics.isSolid)
					rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
				else
					rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
			}
			break;

		case RC_NODE_SHAPE_TYPE_CONE:
			{
				rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_CONE;
				IConeShape* shape = new IConeShape(rc_actor[actor_id].mesh_node, mass, false);

				if(rc_actor[actor_id].physics.isSolid)
					rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
				else
					rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
			}
			break;

		case RC_NODE_SHAPE_TYPE_TRIMESH:
			{
				rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_TRIMESH;
				IBvhTriangleMeshShape* shape;

				if(rc_actor[actor_id].node_type == RC_NODE_TYPE_MESH)
				{
					irr::scene::IAnimatedMeshSceneNode* node = (irr::scene::IAnimatedMeshSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new IBvhTriangleMeshShape(rc_actor[actor_id].mesh_node, node->getMesh(), mass);
				}
				else if(rc_actor[actor_id].node_type == RC_NODE_TYPE_OTMESH)
				{
					irr::scene::IOctreeSceneNode* node = (irr::scene::IOctreeSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new IBvhTriangleMeshShape(rc_actor[actor_id].mesh_node, node->getMesh(), mass);
				}
				else if(rc_actor[actor_id].node_type == RC_NODE_TYPE_TERRAIN)
				{
					irr::scene::ITerrainSceneNode* node = (irr::scene::ITerrainSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new IBvhTriangleMeshShape(rc_actor[actor_id].mesh_node, node->getMesh(), mass);
				}
				//else if(rc_actor[actor_id].node_type == RC_NODE_TYPE_WATER)
					//shape = new IBvhTriangleMeshShape(rc_actor[actor_id].mesh_node, (irr::scene::IAnimatedMeshSceneNode*)rc_actor[actor_id].mesh_node->getMesh(), mass);

				if(rc_actor[actor_id].physics.isSolid)
					rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
				else
					rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
			}
			break;

		case RC_NODE_SHAPE_TYPE_CONVEXHULL:
			{
				rc_actor[actor_id].physics.shape_type = RC_NODE_SHAPE_TYPE_CONVEXHULL;
				IConvexHullShape* shape;

				if(rc_actor[actor_id].node_type == RC_NODE_TYPE_MESH)
				{
					irr::scene::IAnimatedMeshSceneNode* node = (irr::scene::IAnimatedMeshSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new IConvexHullShape(rc_actor[actor_id].mesh_node, node->getMesh(), mass);
				}
				else if(rc_actor[actor_id].node_type == RC_NODE_TYPE_OTMESH)
				{
					irr::scene::IOctreeSceneNode* node = (irr::scene::IOctreeSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new IConvexHullShape(rc_actor[actor_id].mesh_node, node->getMesh(), mass);
				}
				else if(rc_actor[actor_id].node_type == RC_NODE_TYPE_TERRAIN)
				{
					irr::scene::ITerrainSceneNode* node = (irr::scene::ITerrainSceneNode*)rc_actor[actor_id].mesh_node;
					shape = new IConvexHullShape(rc_actor[actor_id].mesh_node, node->getMesh(), mass);
				}

				if(rc_actor[actor_id].physics.isSolid)
					rc_actor[actor_id].physics.rigid_body = rc_physics3D.world->addRigidBody(shape);
				else
					rc_actor[actor_id].physics.ghost = rc_physics3D.world->addGhostObject(shape);
			}
			break;

		default:
			std::cout << "SetActorCollisionShape Error: Invalid shape_type parameter" << std::endl;
	}

	if(rc_actor[actor_id].physics.rigid_body)
	{
		rc_actor[actor_id].physics.rigid_body->getIdentification()->setId(actor_id);
		rc_actor[actor_id].physics.rigid_body->getPointer()->setActivationState(ACTIVE_TAG);
		rc_actor[actor_id].physics.rigid_body->getPointer()->setActivationState(DISABLE_DEACTIVATION);
	}
	else if(rc_actor[actor_id].physics.ghost)
		rc_actor[actor_id].physics.ghost->getIdentification()->setId(actor_id);
}

int rc_getActorCollisionShape(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(!rc_actor[actor].mesh_node)
        return 0;

	return rc_actor[actor].physics.shape_type;
}


void rc_setActorSolid(int actor_id, bool flag)
{
	if(actor_id < 0 || actor_id >= rc_actor.size())
        return;

    if(!rc_actor[actor_id].mesh_node)
        return;

	//I will figure this out eventually
	if(flag != rc_actor[actor_id].physics.isSolid)
	{
		rc_actor[actor_id].physics.isSolid = flag;
		rc_setActorCollisionShape(actor_id, rc_actor[actor_id].physics.shape_type, rc_actor[actor_id].physics.mass);
	}
}

bool rc_getActorCollision(int actor1, int actor2)
{
	int numManifolds = rc_physics3D.world->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        //btPersistentManifold* contactManifold =  rc_physics3D.world->getCollisionCallback(i)->getContactPoint();
        //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
        //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

        irr::u32 a1 = rc_physics3D.world->getCollisionCallback(i)->getBody0()->getIdentification()->getId();
        irr::u32 a2 = rc_physics3D.world->getCollisionCallback(i)->getBody1()->getIdentification()->getId();

       //... here you can check for obA´s and obB´s user pointer again to see if the collision is alien and bullet and in that case initiate deletion.
       //std::cout << "collision found (" << a1 << ", " << a2 << ")" << std::endl;
    }

    return false;
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

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		actor_transform.setTranslation( irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.ghost->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setPosition(actor_transform.getTranslation());
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		actor_transform.setTranslation( irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.rigid_body->clearForces();
		rc_actor[actor].physics.rigid_body->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setPosition(actor_transform.getTranslation());

	}
}

//translate actor from local orientation
void rc_translateActor(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		actor_transform.setTranslation( actor_transform.getTranslation() + irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.ghost->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setPosition(actor_transform.getTranslation());
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		actor_transform.setTranslation( actor_transform.getTranslation() + irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.rigid_body->clearForces();
		rc_actor[actor].physics.rigid_body->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setPosition(actor_transform.getTranslation());
	}
}

//get actor position
void rc_getActorPosition(int actor, double* x, double* y, double* z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		*x = actor_transform.getTranslation().X;
		*y = actor_transform.getTranslation().Y;
		*z = actor_transform.getTranslation().Z;
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		*x = actor_transform.getTranslation().X;
		*y = actor_transform.getTranslation().Y;
		*z = actor_transform.getTranslation().Z;
	}
}

//set actor scale
void rc_setActorScale(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		actor_transform.setScale( irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.ghost->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setScale(actor_transform.getScale());
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		actor_transform.setScale( irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.rigid_body->clearForces();
		rc_actor[actor].physics.rigid_body->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setScale(actor_transform.getScale());
	}

}

//scale actor
void rc_scaleActor(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		actor_transform.setScale( actor_transform.getScale() * irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.ghost->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setScale(actor_transform.getScale());
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		actor_transform.setScale( actor_transform.getScale() * irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.rigid_body->clearForces();
		rc_actor[actor].physics.rigid_body->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setScale(actor_transform.getScale());
	}
}

//get actor scale
void rc_getActorScale(int actor, double* x, double* y, double* z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		*x = actor_transform.getScale().X;
		*y = actor_transform.getScale().Y;
		*z = actor_transform.getScale().Z;
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		*x = actor_transform.getScale().X;
		*y = actor_transform.getScale().Y;
		*z = actor_transform.getScale().Z;
	}
}


//set actor rotation
void rc_setActorRotation(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		actor_transform.setRotationDegrees( irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.ghost->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setRotation( actor_transform.getRotationDegrees() );
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		actor_transform.setRotationDegrees( irr::core::vector3df(x, y, z) );
		rc_actor[actor].physics.rigid_body->clearForces();
		rc_actor[actor].physics.rigid_body->setWorldTransform(actor_transform);
		rc_actor[actor].mesh_node->setRotation( actor_transform.getRotationDegrees() );
	}
}

//rotate actor
void rc_rotateActor(int actor, double x, double y, double z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();

		irr::core::matrix4 m;
		m.setRotationDegrees( actor_transform.getRotationDegrees() );
		irr::core::matrix4 n;
		n.setRotationDegrees( irr::core::vector3df(x,y,z) );
		m *= n;

		rc_actor[actor].physics.ghost->setWorldTransform(m);
		rc_actor[actor].mesh_node->setRotation( m.getRotationDegrees() );
		//rc_actor[actor].mesh_node->updateAbsolutePosition();
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();

		irr::core::matrix4 m;
		m.setRotationDegrees( actor_transform.getRotationDegrees() );
		irr::core::matrix4 n;
		n.setRotationDegrees( irr::core::vector3df(x,y,z) );
		m *= n;

		rc_actor[actor].physics.rigid_body->clearForces();
		rc_actor[actor].physics.rigid_body->setWorldTransform(m);
		rc_actor[actor].mesh_node->setRotation( m.getRotationDegrees() );
		//rc_actor[actor].mesh_node->updateAbsolutePosition();
	}
}

//get actor position
void rc_getActorRotation(int actor, double* x, double* y, double* z)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

	*x = 0;
	*y = 0;
	*z = 0;

    if(rc_actor[actor].physics.ghost)
	{
		//std::cout << "Set GHST POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.ghost->getWorldTransform();
		*x = actor_transform.getRotationDegrees().X;
		*y = actor_transform.getRotationDegrees().Y;
		*z = actor_transform.getRotationDegrees().Z;
	}
	else if(rc_actor[actor].physics.rigid_body)
	{
		//std::cout << "Set POS" << std::endl;
		irr::core::matrix4 actor_transform = rc_actor[actor].physics.rigid_body->getWorldTransform();
		*x = actor_transform.getRotationDegrees().X;
		*y = actor_transform.getRotationDegrees().Y;
		*z = actor_transform.getRotationDegrees().Z;
	}
}

void rc_setActorGravity(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setGravity(irr::core::vector3df(x, y, z));
	}
}

void rc_getActorGravity(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

	*x = 0;
	*y = 0;
	*z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df gvec = rc_actor[actor].physics.rigid_body->getGravity();
		*x = gvec.X;
		*y = gvec.Y;
		*z = gvec.Z;
	}
}

void rc_setActorDamping(int actor, double lin_damping, double ang_damping)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setDamping(lin_damping, ang_damping);
	}
}

double rc_getActorLinearDamping(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		return rc_actor[actor].physics.rigid_body->getLinearDamping();
	}
	return 0;
}

double rc_getActorAngularDamping(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		return rc_actor[actor].physics.rigid_body->getAngularDamping();
	}
	return 0;
}

double rc_getActorLinearSleepThreshold(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		return rc_actor[actor].physics.rigid_body->getLinearSleepingThreshold();
	}
	return 0;
}

double rc_getActorAngularSleepThreshold(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		return rc_actor[actor].physics.rigid_body->getAngularSleepingThreshold();
	}
	return 0;
}

void rc_applyActorDamping(int actor, double timeStep)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyDamping(timeStep);
	}
}

void rc_setActorMassProperties(int actor, double mass, double inertia_x, double inertia_y, double inertia_z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setMassProps(mass, irr::core::vector3df(inertia_x, inertia_y, inertia_z));
	}
}

void rc_getActorLinearFactor(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df lf = rc_actor[actor].physics.rigid_body->getLinearFactor();
		*x = lf.X;
		*y = lf.Y;
		*z = lf.Z;
	}
}

void rc_setActorLinearFactor(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setLinearFactor(irr::core::vector3df(x, y, z));
	}
}

double rc_getActorInverseMass(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		return rc_actor[actor].physics.rigid_body->getInvMass();
	}
	return 0;
}

void rc_integrateActorVelocities(int actor, double step)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->integrateVelocities(step);
	}
}

void rc_applyActorCentralForceLocal(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyCentralForce(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_applyActorCentralForceWorld(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyCentralForce(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_getActorTotalForce(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df f = rc_actor[actor].physics.rigid_body->getTotalForce();
		*x = f.X;
		*y = f.Y;
		*z = f.Z;
	}
}

void rc_getActorTotalTorque(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df f = rc_actor[actor].physics.rigid_body->getTotalTorque();
		*x = f.X;
		*y = f.Y;
		*z = f.Z;
	}
}

void rc_getActorInverseInertiaDiagLocal(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df f = rc_actor[actor].physics.rigid_body->getInvInertiaDiagLocal();
		*x = f.X;
		*y = f.Y;
		*z = f.Z;
	}
}

void rc_setActorInverseInertiaDiagLocal(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setInvInertiaDiagLocal(irr::core::vector3df(x,y,z));
	}
}

void rc_setActorSleepThresholds(int actor, double linear, double angular)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setSleepingThresholds(linear, angular);
	}
}

void rc_applyActorTorqueLocal(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyTorque(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_applyActorTorqueWorld(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyTorque(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_applyActorForceLocal(int actor, double x, double y, double z, double rel_x, double rel_y, double rel_z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyForce(irr::core::vector3df(x,y,z), irr::core::vector3df(rel_x, rel_y, rel_z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_applyActorForceWorld(int actor, double x, double y, double z, double rel_x, double rel_y, double rel_z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyForce(irr::core::vector3df(x,y,z), irr::core::vector3df(rel_x, rel_y, rel_z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_applyActorCentralImpulseLocal(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyCentralImpulse(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_applyActorCentralImpulseWorld(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyCentralImpulse(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_applyActorTorqueImpulseLocal(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyTorqueImpulse(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_applyActorTorqueImpulseWorld(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyTorqueImpulse(irr::core::vector3df(x,y,z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_applyActorImpulseLocal(int actor, double x, double y, double z, double rel_x, double rel_y, double rel_z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyImpulse(irr::core::vector3df(x,y,z), irr::core::vector3df(rel_x, rel_y, rel_z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_applyActorImpulseWorld(int actor, double x, double y, double z, double rel_x, double rel_y, double rel_z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->applyImpulse(irr::core::vector3df(x,y,z), irr::core::vector3df(rel_x, rel_y, rel_z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_clearActorForces(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->clearForces();
	}
}

void rc_updateActorInertiaTensor(int actor)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->updateInertiaTensor();
	}
}

void rc_getActorCOMPosition(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df pos = rc_actor[actor].physics.rigid_body->getCenterOfMassPosition();
		*x = pos.X;
		*y = pos.Y;
		*z = pos.Z;
	}
}

void rc_getActorRotationQ(int actor, double* x, double* y, double* z, double* w)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;
    *w = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::quaternion q = rc_actor[actor].physics.rigid_body->getOrientation();
		*x = q.X;
		*y = q.Y;
		*z = q.Z;
		*w = q.W;
	}
}

void rc_getActorLinearVelocity(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df pos = rc_actor[actor].physics.rigid_body->getLinearVelocity();
		*x = pos.X;
		*y = pos.Y;
		*z = pos.Z;
	}
}

void rc_getActorAngularVelocity(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df pos = rc_actor[actor].physics.rigid_body->getAngularVelocity();
		*x = pos.X;
		*y = pos.Y;
		*z = pos.Z;
	}
}

void rc_setActorLinearVelocityLocal(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setLinearVelocity(irr::core::vector3df(x, y, z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_setActorLinearVelocityWorld(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setLinearVelocity(irr::core::vector3df(x, y, z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_setActorAngularVelocityLocal(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setAngularVelocity(irr::core::vector3df(x, y, z), ERBTransformSpace::ERBTS_LOCAL);
	}
}

void rc_setActorAngularVelocityWorld(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setAngularVelocity(irr::core::vector3df(x, y, z), ERBTransformSpace::ERBTS_WORLD);
	}
}

void rc_getActorLocalPointVelocity(int actor, double rel_x, double rel_y, double rel_z, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df pos = rc_actor[actor].physics.rigid_body->getVelocityInLocalPoint(irr::core::vector3df(rel_x, rel_y, rel_z));
		*x = pos.X;
		*y = pos.Y;
		*z = pos.Z;
	}
}

void rc_getActorLinearVelocityLocal(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		btVector3 v = rc_actor[actor].physics.rigid_body->getPointer()->getWorldTransform().getBasis().transpose() * rc_actor[actor].physics.rigid_body->getPointer()->getLinearVelocity();
		*x = v.getX();
		*y = v.getY();
		*z = v.getZ();
	}
}

void rc_getActorAngularVelocityLocal(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *x = 0;
    *y = 0;
    *z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		btVector3 v = rc_actor[actor].physics.rigid_body->getPointer()->getWorldTransform().getBasis().transpose() * rc_actor[actor].physics.rigid_body->getPointer()->getAngularVelocity();
		*x = v.getX();
		*y = v.getY();
		*z = v.getZ();
	}
}

void rc_getActorAABB(int actor, double* min_x, double* min_y, double* min_z, double* max_x, double* max_y, double* max_z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    *min_x = 0;
	*min_y = 0;
	*min_z = 0;
	*max_x = 0;
	*max_y = 0;
	*max_z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df min_aabb;
		irr::core::vector3df max_aabb;
		rc_actor[actor].physics.rigid_body->getAabb(min_aabb, max_aabb);
		*min_x = min_aabb.X;
		*min_y = min_aabb.Y;
		*min_z = min_aabb.Z;
		*max_x = max_aabb.X;
		*max_y = max_aabb.Y;
		*max_z = max_aabb.Z;
	}
}

double rc_computeActorImpulseDenominator(int actor, double pos_x, double pos_y, double pos_z, double normal_x, double normal_y, double normal_z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		return rc_actor[actor].physics.rigid_body->computeImpulseDenominator(irr::core::vector3df(pos_x, pos_y, pos_z), irr::core::vector3df(normal_x, normal_y, normal_z));
	}
}

double rc_computeActorAngularImpulseDenominator(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		return rc_actor[actor].physics.rigid_body->computeAngularImpulseDenominator(irr::core::vector3df(x, y, z));
	}
}

void rc_setActorAngularFactor(int actor, double x, double y, double z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

    if(rc_actor[actor].physics.rigid_body)
	{
		rc_actor[actor].physics.rigid_body->setAngularFactor(irr::core::vector3df(x, y, z));
	}
}

void rc_getActorAngularFactor(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

	*x = 0;
	*y = 0;
	*z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		irr::core::vector3df af = rc_actor[actor].physics.rigid_body->getAngularFactor();
		*x = af.X;
		*y = af.Y;
		*z = af.Z;
	}
}

void rc_computeActorGyroImpulseLocal(int actor, double step, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

	*x = 0;
	*y = 0;
	*z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		btVector3 v = rc_actor[actor].physics.rigid_body->getPointer()->computeGyroscopicImpulseImplicit_Body(step);
		*x = v.getX();
		*y = v.getY();
		*z = v.getZ();
	}
}

void rc_computeActorGyroImpulseWorld(int actor, double dt, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

	*x = 0;
	*y = 0;
	*z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		btVector3 v = rc_actor[actor].physics.rigid_body->getPointer()->computeGyroscopicImpulseImplicit_World(dt);
		*x = v.getX();
		*y = v.getY();
		*z = v.getZ();
	}
}

void rc_getActorLocalInertia(int actor, double* x, double* y, double* z)
{
	if(actor < 0 || actor >= rc_actor.size())
        return;

	*x = 0;
	*y = 0;
	*z = 0;

    if(rc_actor[actor].physics.rigid_body)
	{
		btVector3 v = rc_actor[actor].physics.rigid_body->getPointer()->getLocalInertia();
		*x = v.getX();
		*y = v.getY();
		*z = v.getZ();
	}
}

void rc_setWorld3DDeltaTime(double dt)
{
	rc_physics3D.DeltaTime = dt;
}

void rc_setWorld3DMaxSubSteps(double steps)
{
	rc_physics3D.maxSubSteps = steps;
}

void rc_setWorld3DTimeStep(double ts)
{
	rc_physics3D.fixedTimeStep = ts;
}


//set actor animation [TODO]
void rc_setActorAnimation(int actor, int start_frame, int end_frame)
{
    if(actor < 0 || actor >= rc_actor.size())
        return;

    switch(rc_actor[actor].node_type)
    {
        case RC_NODE_TYPE_MESH:
            irr::scene::IAnimatedMeshSceneNode* node = (irr::scene::IAnimatedMeshSceneNode*)rc_actor[actor].mesh_node;
            node->setFrameLoop((irr::s32)start_frame, (irr::s32)end_frame );
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
            irr::scene::IAnimatedMeshSceneNode* node = (irr::scene::IAnimatedMeshSceneNode*)rc_actor[actor].mesh_node;
            node->setAnimationSpeed( (irr::f32)speed );
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

void rc_translateCameraW(double x, double y, double z)
{
    if(!(rc_active_canvas > 0 && rc_active_canvas < rc_canvas.size()))
        return;

    rc_canvas[rc_active_canvas].camera.translateW(x, y, z);
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
