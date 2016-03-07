#ifndef SIMOBJECT_H
#define SIMOBJECT_H
#include "vector3d.h"
#include "irrlicht.h"
#include "Logger.h"
#include "DataStorage.h"
#include <string>
#include <math.h>
#include <cmath>

/*
 * Parent class for each simulated object
 */
class SimObject
{
public:
    /*
     * The string is the name/tag of the object <br>
     * The ISceneNode is the model of the object
     */
    SimObject(std::string, irr::scene::ISceneNode*);
    void update();
    void reset();

    void setAcc(irr::core::vector3df);
    irr::core::vector3df getAcc();
    irr::core::vector3df getPos();

    void setRot(irr::core::vector3df);
    irr::core::vector3df getRot();

    std::string getName();

protected:
    std::string name;
    irr::scene::ISceneNode *node = 0;

    irr::core::vector3df vel;
    irr::core::vector3df acc;
    irr::core::vector3df fri;
    float friction = 0.5f;

};

#endif // SIMOBJECT_H
