#ifndef SUB_H
#define SUB_H
#include "SimObject.h"
#include "irrlicht.h"
#include "../InputHandler.h"

class SimSub: public SimObject
{
public:
    SimSub(std::string name, irr::scene::ISceneNode* n, InputHandler* ih);
    void setCam(irr::scene::ICameraSceneNode*);
    void update();
    irr::scene::ICameraSceneNode* getSceneNode();

private:
    irr::scene::ICameraSceneNode* cam = 0;
    InputHandler* ih = 0;
};

#endif // SUB_H
