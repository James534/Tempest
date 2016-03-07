#include "Sub.h"

Sub::Sub(std::string name, irr::scene::ISceneNode* n):SimObject(name, n)
{

}

void Sub::setCam(irr::scene::ICameraSceneNode *c){
    cam = c;
}

/*void Sub::update()
{
    SimObject::update();
    cam->setPosition(node->getPosition());
}*/
