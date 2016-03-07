#include "SimObject.h"

SimObject::SimObject(std::string name, irr::scene::ISceneNode *n)
{
    this->name = name;
    node = n;
    n->setMaterialFlag(irr::video::EMF_LIGHTING, false);

    fri.X = -1;
    fri.Y = -1;
    fri.Z = -1;
}

void SimObject::update(){
    irr::core::vector3df pos = node->getPosition();

    if (acc.getLength() > 0){
        vel += acc;
        //acc += fri;
        std::string msg = std::to_string(acc.X) + ' ' + std::to_string(acc.Y) + ' ' + std::to_string(acc.Z) + " Act";
        //Logger::Log(msg);
        if (fabs(acc.X)-friction < 0)
            acc.X = 0;
        else
            acc.X -= std::copysign(friction, acc.X);
        if (fabs(acc.Y)-friction < 0)
            acc.Y = 0;
        else
            acc.Y -= std::copysign(friction, acc.Y);
        if (fabs(acc.Z)-friction < 0)
            acc.Z = 0;
        else
            acc.Z -= std::copysign(friction, acc.Z);
    }
    if (vel.getLength() > 0){
        if (vel.getLength() > 5){
            vel = vel.normalize()*5;
        }
        pos += vel;
        std::string msg = std::to_string(vel.X) + ' ' + std::to_string(vel.Y) + ' ' + std::to_string(vel.Z) + " Act";
        //Logger::Log(msg);
        if (fabs(vel.X)-friction/10.0f< 0){
            vel.X = 0;
        }else{
            vel.X += std::copysign(friction/10.0f, -vel.X);
        }

        if (fabs(vel.Y)-friction/10.0f < 0){
            vel.Y = 0;
        }else{
            vel.Y += std::copysign(friction/10.0f, -vel.Y);
        }

        if (fabs(vel.Z)-friction/10.0f < 0){
            vel.Z = 0;
        }else{
            vel.Z += std::copysign(friction/10.0f, -vel.Z);
        }

        node->setPosition(pos);
        node->setRotation(irr::core::vector3df(0,0,0));
    }
    //std::string msg = std::to_string(acc.X) + ' ' + std::to_string(acc.Y) + ' ' + std::to_string(acc.Z) + " Act";
    //Logger::Log(msg);
}

void SimObject::reset(){
    this->acc.X = 0;
    this->acc.Y = 0;
    this->acc.Z = 0;
    this->vel.X = 0;
    this->vel.Y = 0;
    this->vel.Z = 0;
    node->setPosition(irr::core::vector3df(0,0,0));
}

void SimObject::setAcc(irr::core::vector3df a){
    this->acc.X = a.X;
    this->acc.Y = a.Y;
    this->acc.Z = a.Z;
    std::string msg = std::to_string(acc.X) + ' ' + std::to_string(acc.Y) + ' ' + std::to_string(acc.Z);
    //Logger::Log(msg);
}
void SimObject::setRot(irr::core::vector3df r){
    node->setRotation(r);
}

irr::core::vector3df SimObject::getAcc(){
    return acc;
}
irr::core::vector3df SimObject::getPos(){
    return node->getPosition();
}
irr::core::vector3df SimObject::getRot(){
    return node->getRotation();
}

std::string SimObject::getName(){
    return name;
}
