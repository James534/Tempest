#ifndef SIM_H
#define SIM_H

#include "irrlicht.h"
#include "Objects/SimObject.h"
#include "Objects/Buoy.h"
#include "Objects/SimSub.h"
#include "SimLogger.h"
#include "InputHandler.h"
#include "DataStorage.h"
#include <vector>
#include <opencv2/opencv.hpp>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

struct dataCap{
    cv::Mat* frame;
    InputHandler* ih;
    bool* runSim;
    bool* threadAlive;
};

/*
 * The main class of the Simulator
 */
class Sim
{
public:
    //Sim(cv::Mat*, InputHandler*, bool*, bool*);
    Sim(void*);

    /*
     * Starts running the simulator
     */
    int start();

private:
    //Irrlicht stuff
    IrrlichtDevice *device = 0;
    IVideoDriver* driver = 0;
    ISceneManager* smgr = 0;
    IGUIEnvironment* guienv = 0;
    ISceneNodeAnimatorCollisionResponse* anim = 0;

    ICameraSceneNode* cameras[4] = {0,0,0,0};
    ISceneNode* camChilds[4] = {0,0,0,0};
    int resX = 640;
    int resY = 480;

    InputHandler* ih = 0;                //handles input from user
    std::vector<SimObject*> objs;   //list of simulated objects

    //current frame in cv::Mat
    cv::Mat *frame;
    dataCap* cap;
    bool* runSim;
    bool* threadAlive;
};

#endif // SIM_H
