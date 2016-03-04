#include "Sim.h"
#include "btBulletDynamicsCommon.h"
#include <cstdlib>

//Declare global variables for physical world and objects
static btDiscreteDynamicsWorld *World;
static core::list<btRigidBody *> Objects;

Sim::Sim()
{
    /**
     * Setup Irrlicht stuff
     */
    //device = createDevice( video::EDT_OPENGL, dimension2d<u32>(1280, 960), 16,
      //      false, true, false, &ih);
    SIrrlichtCreationParameters params = SIrrlichtCreationParameters();
    params.AntiAlias = 8;
    params.DriverType = video::EDT_OPENGL;
    params.WindowSize = core::dimension2d<u32>(1280, 960);
    device = createDeviceEx(params);
    if (!device){
        Logger::Log("FATAL- Could not create device");
        return;
    }
    device->setWindowCaption(L"MDA Simulator 0.1");

    driver = device->getVideoDriver();
    smgr = device->getSceneManager();
    guienv = device->getGUIEnvironment();

    /**
     * Load textures into the map
     */
    DataStorage::loadTextures(driver);

    /**
     * Loads the nodes into the scene
     */
    ISceneNode *b = smgr->addSphereSceneNode();
    if (!b){
        Logger::Log("Could not create sphere node");
    }else{
        Buoy *ball = new Buoy("ball", b);
        objs.push_back(ball);
    }

    //Light and Fog
    ILightSceneNode* light1 = smgr->addLightSceneNode( 0, core::vector3df(0,500,0), video::SColorf(0.3f,0.3f,0.3f), 50000.0f, 1 );
    driver->setFog(video::SColor(0, 120,140,160), video::EFT_FOG_LINEAR, 20, 250, .001f, false, false);
    ISceneNode * scenenode = smgr->getRootSceneNode();
    scenenode->setMaterialFlag(EMF_FOG_ENABLE, true);
    light1->setMaterialFlag(EMF_FOG_ENABLE, true);

    //Load obstacles (need to be separated so that buoys can move)
    IAnimatedMesh* mesh = smgr->getMesh("assets/obstacles.3ds");
    IMeshSceneNode * node = 0;
    if (mesh)
        node = smgr->addOctreeSceneNode(mesh->getMesh(0), 0);

    if (node)
    {
        node->setMaterialFlag(EMF_FOG_ENABLE, true);
        node->setMaterialFlag(EMF_LIGHTING, true);
        node->setScale(core::vector3df(20,20,20));
    }

    IAnimatedMesh* roomMesh = smgr->getMesh("assets/stadium.3ds");
    IMeshSceneNode * roomNode = 0;
    if (roomMesh)
        roomNode = smgr->addOctreeSceneNode(roomMesh->getMesh(0), 0);


    if (roomNode)
    {
        roomNode->setMaterialFlag(EMF_FOG_ENABLE, true);
        roomNode->setMaterialFlag(EMF_LIGHTING, true);
        roomNode->setScale(core::vector3df(20,20,20));
    }

    scene::ITriangleSelector* selector = 0;

    if (node)
    {
        node->setPosition(core::vector3df(0,0,0));

        selector = smgr->createOctreeTriangleSelector(
                node->getMesh(), node, 128);
        node->setTriangleSelector(selector);
        // We're not done with this selector yet, so don't drop it.
    }

    ICameraSceneNode* camera = smgr->addCameraSceneNodeFPS(0, 100.0f, 0.05f);

    device->getCursorControl()->setVisible(false);
    anim = smgr->createCollisionResponseAnimator(
    selector, camera, vector3df(5,5,5),
    vector3df(0,0,0), vector3df(0,0,0));

    if (selector)
    {
        selector->drop(); // As soon as we're done with the selector, drop it.
        camera->addAnimator(anim);
        //anim->drop();  // And likewise, drop the animator when we're done referring to it.
    }


}

//Sim::~Sim(){

//}

int Sim::start(){
    //if the device cannot be created, just exit the program
    if (!device)
        return 1;    

    guienv->addStaticText(L"Hello World!",
        rect<s32>(10,10,260,22), true);

    //Add camera node to a static position. Need to change later to have this on the sub
    //smgr->addCameraSceneNode(0, vector3df(0,30,-40), vector3df(0,5,0));

    bool collision;

    // In order to do framerate independent movement, we have to know
    // how long it was since the last frame
    u32 then = device->getTimer()->getTime();

    while(device->run())
    {
        // Work out a frame delta time.
        const u32 now = device->getTimer()->getTime();
        const f32 frameDeltaTime = (f32)(now - then) / 1000.f; // Time in seconds
        then = now;

        //Initialize bullet
        btDefaultCollisionConfiguration *CollisionConfiguration = new btDefaultCollisionConfiguration();
        btBroadphaseInterface *BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
        btCollisionDispatcher *Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
        btSequentialImpulseConstraintSolver *Solver = new btSequentialImpulseConstraintSolver();
        World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
        
        UpdatePhysics(DeltaTime);

        //temp stuff for testing
        SimObject *b = objs.at(0);
        vector3df acc = b->getAcc();
        std::string msg = std::to_string(acc.X) + ' ' + std::to_string(acc.Y)
                + ' ' + std::to_string(acc.Z) + " Sim";
        Logger::Log(msg);

        //input processing
        if(ih.IsKeyDown(irr::KEY_KEY_W)){
            acc.Y += 5 * frameDeltaTime;
        }
        else if(ih.IsKeyDown(irr::KEY_KEY_S))
            acc.Y -= 5 * frameDeltaTime;
        if(ih.IsKeyDown(irr::KEY_KEY_A))
            acc.X -= 5 * frameDeltaTime;
        else if(ih.IsKeyDown(irr::KEY_KEY_D))
            acc.X += 5 * frameDeltaTime;

        //std::string msg = "SIM " + std::to_string(acc.X) + ' ' + std::to_string(acc.Y) + ' ' + std::to_string(acc.Z);
        //Logger::Log(msg);
        b->setAcc(acc);

        for (SimObject *so: objs){
            so->setAcc(acc);
            so->act();
        }
        //collision check
        collision = Sim::anim->collisionOccurred();
        if (collision)
            Logger::Log("collision");

        driver->beginScene(true, true, SColor(255,100,101,140));

        smgr->drawAll();
        guienv->drawAll();

        driver->endScene();
    }
    device->closeDevice();
    return 0;
}

void CreateBuoy(const btVector3 &TPosition, btScalar TRadius, btScalar TMass){
    ISceneNode *b = smgr->addSphereSceneNode();
    if (!b) {
        Logger::Log("Could not create sphere node");
    }
    else {
        Buoy *ball = new Buoy("ball", b);
        objs.push_back(ball);
    }

    // Set the initial position of the object
    btTransform Transform;
    Transform.setIdentity();
    Transform.setOrigin(TPosition);

    // Give it a default MotionState
    btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

    // Create the shape
    btCollisionShape *Shape = new btSphereShape(TRadius);

    // Add mass
    btVector3 LocalInertia;
    Shape->calculateLocalInertia(TMass, LocalInertia);

    // Create the rigid body object
    btRigidBody *RigidBody = new btRigidBody(TMass, MotionState, Shape, LocalInertia);

    // Store a pointer to the irrlicht node so we can update it later
    RigidBody->setUserPointer((void *)(Node));

    // Add it to the world
    World->addRigidBody(RigidBody);
    Objects.push_back(RigidBody);
}

// Runs the physics simulation.
// - TDeltaTime tells the simulation how much time has passed since the last frame so the simulation can run independently of the frame rate.
void UpdatePhysics(u32 TDeltaTime) {

    World->stepSimulation(TDeltaTime * 0.001f, 60);

    btRigidBody *TObject;
    // Relay the object's orientation to irrlicht
    for(core::list<btRigidBody *>::Iterator it = Objects.begin(); it != Objects.end(); ++it) {

        //UpdateRender(*Iterator);
        scene::ISceneNode *Node = static_cast<scene::ISceneNode *>((*it)->getUserPointer());
        TObject = *it;

        // Set position
        btVector3 Point = TObject->getCenterOfMassPosition();
        Node->setPosition(core::vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

        // Set rotation
        btVector3 EulerRotation;
        QuaternionToEuler(TObject->getOrientation(), EulerRotation);
        Node->setRotation(core::vector3df(EulerRotation[0], EulerRotation[1], EulerRotation[2]));

    }
}

void UpdateRender(btRigidBody *TObject) {
    ISceneNode *Node = static_cast<ISceneNode *>(TObject->getUserPointer());

    // Set position
    btVector3 Point = TObject->getCenterOfMassPosition();
    Node->setPosition(vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

    // Set rotation
    vector3df Euler;
    const btQuaternion& TQuat = TObject->getOrientation();
    quaternion q(TQuat.getX(), TQuat.getY(), TQuat.getZ(), TQuat.getW());
    q.toEuler(Euler);
    Euler *= RADTODEG;
    Node->setRotation(Euler);
}

// Converts a quaternion to an euler angle
void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler) {
    btScalar W = TQuat.getW();
    btScalar X = TQuat.getX();
    btScalar Y = TQuat.getY();
    btScalar Z = TQuat.getZ();
    float WSquared = W * W;
    float XSquared = X * X;
    float YSquared = Y * Y;
    float ZSquared = Z * Z;

    TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
    TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
    TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
    TEuler *= core::RADTODEG;
}