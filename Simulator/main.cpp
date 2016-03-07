#include "Sim.h"
#include <opencv2/opencv.hpp>
#include <pthread.h>

pthread_t* clientThread;
pthread_mutex_t* clientLock;

void* run(void* frame, InputHandler* ih){
    Sim sim ((cv::Mat*)frame, ih);
    sim.start();
}

int main()
{
    cv::Mat *f = new cv::Mat(240, 640, CV_8UC3);
    InputHandler* ih = new InputHandler(true);

    run(f, ih);
    /*pthread_t thread;
    int rc = pthread_create(&thread, NULL, run, f);
    if (rc)
           std::cout<<"Couldnt create thread"<<std::endl;
    while (1){
        cv::imshow("Frame", *f);
        if (cv::waitKey() == 27){
            break;
        }
    }
    pthread_exit(NULL);*/
    delete (f);
    delete (ih);
    return 0;
}
