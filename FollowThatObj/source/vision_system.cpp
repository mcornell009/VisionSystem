#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <boost/thread.hpp>
//#include "../externals/boost_1_50_0/boost/thread.hpp"
#include "ObjRec.cpp"
#include "RegionFinding.cpp"
#include <vector>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;
//-lboost_thread-mt  -- need cxx flag for cmake to work

class vision_system
{
    private:
    Mat Limg, Rimg;
    Mat LimgDisp, RimgDisp;
    string LwinName, RwinName;
    bool running, canRun, cameraSwitch, cameraON;
    VideoCapture camera0, camera1;
    ObjectRecognition ObjRec;
    RegionFinding RegFind;
    
    void mainLoop();
    void keyboardControl();
    void displayImages();
    
    public:
    vision_system();
    void getNextImage();
    void start();
    void stop();
};

vision_system::vision_system()
{
    cameraSwitch = true, cameraON = false;       
    //  Init Cameras
    camera0 = VideoCapture(0); 
    camera1 = VideoCapture(1);
    if(camera0.isOpened() && camera1.isOpened()) canRun = true; // check if we succeeded 
    else
    {
    	cout << "Cameras are not attached" << endl;
    	canRun = false;
    }
    
    // Create Windows for Display
    LwinName = "Left Camera", RwinName = "Right Camera";
    namedWindow( LwinName, CV_WINDOW_AUTOSIZE );
    namedWindow( RwinName, CV_WINDOW_AUTOSIZE );

    moveWindow(LwinName.c_str(), 0, 0);
    moveWindow(RwinName.c_str(), 640, 0);

    ObjRec.loadImageDB() ? canRun = false : canRun = true;
    cout << "Running " << canRun << endl;    
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


void vision_system::getNextImage()
{
    Mat _Limg, _Rimg;
    
    while(running)
    {
        if(cameraSwitch)
     	{ 
    	    camera0 >> _Limg;
    	    camera1 >> _Rimg;
        } 
        else
        {
    	    camera0 >> _Rimg;
    	    camera1 >> _Limg;	
        }
        
        _Limg.copyTo(Limg);
        _Rimg.copyTo(Rimg);
        
        cameraON = true;
    }
}

void vision_system::displayImages()
{
    
    imshow( LwinName, LimgDisp );
    imshow( RwinName, RimgDisp );       
    
    

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

void vision_system::mainLoop()
{    
    boost::thread getNextImages(boost::bind(&vision_system::getNextImage,this));
    
    vector<Rect> L_ROIs, R_ROIs;
    vector<Mat> L_ROIMasks, R_ROIMasks;
    Mat _Limg, _Rimg;
    TickMeter tm;       
    while(running)
    {
 
    tm.start();
        
        if (cameraON)
        {
            Limg.copyTo(_Limg);
            Rimg.copyTo(_Rimg);
            
            boost::thread L_RF(boost::bind(&RegionFinding::findRegions, &RegFind, _Limg));
            L_ROIs = RegFind.regions;
            L_ROIMasks = RegFind.masks;
            boost::thread R_RF(boost::bind(&RegionFinding::findRegions, &RegFind, _Rimg));
            R_ROIs = RegFind.regions;
            R_ROIMasks = RegFind.masks;
            L_RF.join();
            R_RF.join();
 


            _Limg.copyTo(LimgDisp);
            _Rimg.copyTo(RimgDisp);
            
            displayImages();
            keyboardControl();
            tm.stop();
            cout << "Full loop in " << tm.getTimeMilli() << " ms" << endl;
            tm.reset();
        }
    }
}

//*/
int main(int argc, char** argv)
{
    vision_system vs;
    vs.start();
}

//*/
