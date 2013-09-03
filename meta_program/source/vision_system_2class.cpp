#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

//#include <pthread.h>
#include"../externals/boost_1_50_0/boost/thread.hpp"

#include "ObjRec.cpp"
#include "RegionFinding.cpp"
#include <vector>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;


/*struct callable
{
    void operator()();
};*/


class img_grabber
{
    private:
    Mat Limg, Rimg;
    bool running, canRun, cameraSwitch;
    VideoCapture camera0, camera1;
    void mainLoop();
    void keyboardControl();
    void displayImages();
    void getNextImage();
    
    public: 
    img_grabber();  
    void start();
    void stop();
    

};


class vision_system
{
    private:
    Mat Limg, Rimg;
    bool running, canRun, cameraSwitch;
    ObjectRecognition ObjRec;
    RegionFinding RegFind;
    
    void mainLoop();
    void keyboardControl();
    void displayImages();
    
    public:
    vision_system();
    //void* getNextImage(void*);
    void start();
    void stop();
};

img_grabber::img_grabber()
{
    
    cameraSwitch = true;
    
    camera0 = VideoCapture(0); 
    camera1 = VideoCapture(1);
    if(camera0.isOpened() && camera1.isOpened()) canRun = true; // check if we succeeded 
    else
    {
    	cout << "Cameras are not attached" << endl;
    	canRun = false;
    }
    cout << "Cameras Initialized" << endl;
    start();

}

void img_grabber::start()
{
    if(canRun)
    {
    running = true;
    mainLoop();
    }
}

void img_grabber::stop()
{
    running = false;
}

void img_grabber::getNextImage()
{
    Mat LNextImg, RNextImg;
    {
        if(cameraSwitch)
     	{ 
    	    camera0 >> LNextImg;
    	    camera1 >> RNextImg;
        } 
        else
        {
    	    camera0 >> RNextImg;
    	    camera1 >> LNextImg;	
        }
        
        LNextImg.copyTo(Limg);
        RNextImg.copyTo(Rimg);
    }
}

void img_grabber::displayImages()
{
    string LwinName = "Left Camera", RwinName = "Right Camera", CwinName = "Control Window";
    
    imshow( LwinName, Limg );
    imshow( RwinName, Rimg );       
    
    //moveWindow(LwinName.c_str(), 0, 0);
    //moveWindow(RwinName.c_str(), 640, 0);

}

void img_grabber::keyboardControl()
{
    switch ( (char)waitKey(5) )
    {
	case 27: case 'q':  case 'Q':
	    running = false;
	    break;
    }
}



void img_grabber::mainLoop()
{    
//    pthread_t thread1, thread2;
//    pthread_create( &thread1, NULL, this.getNextImage, (void*) NULL);
//    boost::thread getNextImages(getNextImage);
    while(running)
    {
        TickMeter tm;
        tm.start();
        getNextImage();
        displayImages();
        keyboardControl();
        
        tm.stop();
        cout << "Updated grabbed image in " << tm.getTimeMilli() << " ms" << endl;
    }
}




vision_system::vision_system()
{
    ObjRec.loadImageDB();
    boost::thread getNextImages(img_grabber);
    //getNextImages.join();
    }
}

void vision_system::start()
{
    if(canRun)
    {
    running = true;
    

    mainLoop();
    }
}

void vision_system::stop()
{
    running = false;
}


void vision_system::displayImages()
{
    string LwinName = "Left Camera", RwinName = "Right Camera", CwinName = "Control Window";
    
    imshow( LwinName, Limg );
    imshow( RwinName, Rimg );       
    
    //moveWindow(LwinName.c_str(), 0, 0);
    //moveWindow(RwinName.c_str(), 640, 0);

}

void vision_system::keyboardControl()
{
    switch ( (char)waitKey(5) )
    {
	case 27: case 'q':  case 'Q':
	    running = false;
	    break;
    }
}

//boost::thread startCameras()
//{
//    img_grabber ig;
//    callable x;
//    return boost::thread(x);
//}

void vision_system::mainLoop()
{    
    

    
    while(running)
    {
        TickMeter tm;
        tm.start();
        
        vector<Rect> L_ROIs, R_ROIs;
        vector<Mat> L_ROIMasks, R_ROIMasks;
        
        RegFind.findRegions(Limg, L_ROIs, L_ROIMasks);
        RegFind.findRegions(Rimg, R_ROIs, R_ROIMasks);
        
        displayImages();
        keyboardControl();
        
        tm.stop();
        cout << "Full loop in " << tm.getTimeMilli() << " ms" << endl;
    }
}
void find_the_question(int the_answer)
{
   cout << the_answer << endl;
}
//*/
int main(int argc, char** argv)
{
//    img_grabber ig;
//    ig.start();
    vision_system vs;
    vs.start();
 //    boost::thread deep_thought_2(find_the_question,42);
}
//*/
