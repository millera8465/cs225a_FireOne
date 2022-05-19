/**
 * @file object.h
 * @brief Helper function to add objects in chai world (without physics)
 * 
 */

#include "Sai2Graphics.h"
#include <typeinfo>
#include <list>

#ifndef _OBJECT_H_
#define _OBJECT_H_

//chai3d::cMesh* fire = new chai3d::cMesh();
auto fire = new chai3d::cMesh();
auto sphere = new chai3d::cMesh();
//chai3d::cMesh* fires [5];
std::list<chai3d::cMesh*> firesList;


void addSphere(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& pos,
                const Quaterniond& ori,
                const double radius,
                const Vector4d& rgba) {

    // initialize a cGenericObject to represent this object in the world
    auto object = new chai3d::cMesh();
    chai3d::cCreateSphere(object, radius, 100, 100);  // last two values are resolution
    object->m_name = name;
    // set object position and rotation
    object->setLocalPos(chai3d::cVector3d(pos(0), pos(1), pos(2)));
    { // brace temp variables to separate scope
        Quaternion<double> tmp_q(ori.w(), ori.x(), ori.y(), ori.z());
        chai3d::cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
        object->setLocalRot(tmp_cmat3);	
    }

    // visuals
    auto color = new chai3d::cColorf(rgba(0), rgba(1), rgba(2), rgba(3));
    object->m_material->setColor(*color);

    // add to world
    graphics->_world->addChild(object);
}

void addSphere2(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& pos,
                const Quaterniond& ori,
                const double radius,
                const Vector4d& rgba) {

    // initialize a cGenericObject to represent this object in the world
    sphere = new chai3d::cMesh();
    chai3d::cCreateSphere(sphere, radius, 100, 100);  // last two values are resolution
    sphere->m_name = name;
    // set object position and rotation
    sphere->setLocalPos(chai3d::cVector3d(pos(0), pos(1), pos(2)));
    { // brace temp variables to separate scope
        Quaternion<double> tmp_q(ori.w(), ori.x(), ori.y(), ori.z());
        chai3d::cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
        sphere->setLocalRot(tmp_cmat3); 
    }

    // visuals
    auto color = new chai3d::cColorf(rgba(0), rgba(1), rgba(2), rgba(3));
    sphere->m_material->setColor(*color);

    // add to world
    graphics->_world->addChild(sphere);
}

void addBox(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& pos,
                const Quaterniond& ori,
                const Vector3d& dim,
                const Vector4d& rgba) {

    // initialize a cGenericObject to represent this object in the world
    auto object = new chai3d::cMesh();
    chai3d::cCreateBox(object, dim(0), dim(1), dim(2));  // last two values are resolution
    object->m_name = name;
    // set object position and rotation
    object->setLocalPos(chai3d::cVector3d(pos(0), pos(1), pos(2)));
    { // brace temp variables to separate scope
        Quaternion<double> tmp_q(ori.w(), ori.x(), ori.y(), ori.z());
        chai3d::cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
        object->setLocalRot(tmp_cmat3);	
    }

    // visuals
    // chai3d::cColorf* color = NULL;
    auto color = new chai3d::cColorf(rgba(0), rgba(1), rgba(2), rgba(3));
    object->m_material->setColor(*color);

    // add to world
    graphics->_world->addChild(object);
}


void increaseRectangularFire(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& pos,
                const Quaterniond& ori,
                const Vector3d& dim,
                const Vector4d& rgba) {
    
    // initialize a cGenericObject to represent this object in the world
    fire = new chai3d::cMesh();


    chai3d::cCreateBox(fire, dim(0), dim(1), dim(2));  // last two values are resolution
    fire->m_name = name;
    // set object position and rotation
    fire->setLocalPos(chai3d::cVector3d(pos(0), pos(1), pos(2)));
    { // brace temp variables to separate scope
        Quaternion<double> tmp_q(ori.w(), ori.x(), ori.y(), ori.z());
        chai3d::cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
        fire->setLocalRot(tmp_cmat3); 
    }

    // visuals
    // chai3d::cColorf* color = NULL;
    auto color = new chai3d::cColorf(rgba(0), rgba(1), rgba(2), rgba(3));
    fire->m_material->setColor(*color);

    // add to world
    graphics->_world->addChild(fire);


}

void removeFire(Sai2Graphics::Sai2Graphics* graphics) {
    graphics->_world->removeChild(fire);    
}
void removeSphere(Sai2Graphics::Sai2Graphics* graphics) {
    graphics->_world->removeChild(sphere);       
}

void addFireToList(const string& name,
                const Vector3d& pos,
                const Quaterniond& ori,
                const Vector3d& dim,
                const Vector4d& rgba) {

    auto object = new chai3d::cMesh();

    chai3d::cCreateBox(object, dim(0), dim(1), dim(2));  // last two values are resolution
    object->m_name = name;
    // set object position and rotation
    object->setLocalPos(chai3d::cVector3d(pos(0), pos(1), pos(2)));
    { // brace temp variables to separate scope
        Quaternion<double> tmp_q(ori.w(), ori.x(), ori.y(), ori.z());
        chai3d::cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
        object->setLocalRot(tmp_cmat3); 
    }

    // visuals
    // chai3d::cColorf* color = NULL;
    auto color = new chai3d::cColorf(rgba(0), rgba(1), rgba(2), rgba(3));
    object->m_material->setColor(*color);

    // add to world
    //graphics->_world->addChild(object);

    // add to list
    firesList.push_back(object);


}

void loadFiresToScene(Sai2Graphics::Sai2Graphics* graphics) {
    /*
    //Create an iterator of std::list
    std::list<chai3d::cMesh*>::iterator it;

    // Make iterate point to begining and incerement it one by one till it reaches the end of list.
    for (it = firesList.begin(); it != firesList.end(); it++)
    {
        // add to world
        graphics->_world->addChild(it);
    }*/


    /*for (const chai3d::cMesh* & _fire : firesList)
    {
        std::cout << "holis" << std::endl;
    }*/

    std::list<chai3d::cMesh*>::iterator it;

    // Make iterate point to begining and incerement it one by one till it reaches the end of list.
    for (it = firesList.begin(); it != firesList.end(); it++)
    {
        // add to world
        graphics->_world->addChild(*it);
    }

}

void removeFires(Sai2Graphics::Sai2Graphics* graphics) {
    std::list<chai3d::cMesh*>::iterator it;

    // Make iterate point to begining and incerement it one by one till it reaches the end of list.
    for (it = firesList.begin(); it != firesList.end(); it++)
    {
        // add to world
        graphics->_world->removeChild(*it);
    }
    firesList.clear();
}



#endif