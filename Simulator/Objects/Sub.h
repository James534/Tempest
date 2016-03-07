#ifndef SUB_H
#define SUB_H
#include "SimObject.h"
#include "irrlicht.h"

class Sub: public SimObject
{
public:
    Sub(std::string, irr::scene::ISceneNode*);
    void setCam(irr::scene::ICameraSceneNode*);
    //void update();
private:
    irr::scene::ICameraSceneNode* cam = 0;
};

#endif // SUB_H
