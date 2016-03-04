/*
#include "irrlicht/irrlicht.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace cv;
*/
#include "Sim.h"
int main()
{
    Sim sim;
    sim.start();

    /**
    IrrlichtDevice *device =
        createDevice( video::EDT_OPENGL, dimension2d<u32>(640, 480), 16,
            false, true, false, 0);

    if (!device)
        return 1;

    device->setWindowCaption(L"Hello World! - Irrlicht Engine Demo");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    IGUIEnvironment* guienv = device->getGUIEnvironment();

    guienv->addStaticText(L"Hello World! This is the Irrlicht Software renderer!",
        rect<s32>(10,10,260,22), true);

    IAnimatedMesh* mesh = smgr->getMesh("sydney.md2");
    if (!mesh)
    {
        device->drop();
        return 1;
    }
    IAnimatedMeshSceneNode* node = smgr->addAnimatedMeshSceneNode( mesh );

    if (node)
    {
        node->setMaterialFlag(EMF_LIGHTING, false);
        node->setMD2Animation(scene::EMAT_STAND);
        node->setMaterialTexture( 0, driver->getTexture("sydney.bmp") );
    }

    smgr->addCameraSceneNode(0, vector3df(0,30,-40), vector3df(0,5,0));

    Mat frame = Mat::zeros(480,640,CV_8UC3);
    //Mat frame = cv::imread("sydney.bmp");


    while(device->run())
    {
        driver->beginScene(true, true, SColor(255,100,101,140));

        smgr->drawAll();
        guienv->drawAll();


        IImage* image = driver->createScreenShot();
        for(int y = 0; y < frame.rows; y++){
            for(int x = 0; x < frame.cols; x++){
                SColor color = image->getPixel(x, y).color;
                Vec3b CVColor(color.getBlue(), color.getGreen(), color.getRed());
                frame.at<Vec3b>(y,x) = CVColor;
            }
        }

        cv::imshow("OpenCV", frame);
        //imwrite("name.jpg", frame);

        driver->endScene();
        image->drop();
        int key = cv::waitKey(1);
        if (key == 27)
            break;
    }
    //driver->drop();
    //smgr->drop();
    //guienv->drop();
    device->closeDevice();
    //device->run();
    //free(device);
    //delete device;
    //device->drop();
    std::cout<<"EOF"<<std::endl;
    */
    return 0;
}

/*
That's it. Compile and run.
**/
