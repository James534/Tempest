#include "SimSub.h"

SimSub::SimSub(std::string name, irr::scene::ISceneNode* n, InputHandler* ih):SimObject(name, n)
{
    this->ih = ih;
}

void SimSub::setCam(irr::scene::ICameraSceneNode *c){
    cam = c;
}

void SimSub::update()
{
    acc = ih->getAcc();
    node->setRotation(ih->getRot());
    //SimObject::update();
    //cam->setPosition(node->getPosition());
    //ih->setAcc(acc);
    //ih->setRot(node->getRotation());
}

irr::scene::ICameraSceneNode* SimSub::getSceneNode(){
    return cam;
}
