#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <string>
#include <iostream>
#include <boost/thread.hpp>

using namespace cv;
using namespace std;

class DBcreator
{
	private:
    Mat Limg, Rimg, LimgDisp, RimgDisp;
    Mat background, forground;
    string LwinName, RwinName;
    bool running, canRun, cameraSwitch, cameraON;
    VideoCapture camera0, camera1;
    
    void mainLoop();
    void keyboardControl();
    void displayImages();    

    public:
    DBcreator();
    void getNextImage();
    void start();
    void stop();

};

DBcreator::DBcreator()
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

    canRun ? cout << "Looking Good, Can Run, starting program" << endl : cout << "Cannot run (canRun bool false), exiting program" << endl;  
}

void DBcreator::start()
{

    if(canRun)
    {
    running = true;
    mainLoop();
    }
    else stop();
}

void DBcreator::stop()
{
    running = false;
}


void DBcreator::getNextImage()
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

void DBcreator::displayImages()
{
    imshow( LwinName, LimgDisp );
    imshow( RwinName, RimgDisp );       
}

void DBcreator::keyboardControl()
{
	Mat diff, _forground;
//    Mat _Limg, _Rimg;
//    Limg.copyTo(_Limg);
//    Rimg.copyTo(_Rimg);
    switch ( (char)waitKey(100) )
    {
	case 27: case 'q':  case 'Q':
	    running = false;
	    break;
	case 'b': case 'B':
	    Limg.copyTo(background);
	    imshow("Background", background);
	    break;
	case 'z': case 'Z':
	    if (!background.empty())
	    {
	    \
//	    forground = Limg;
	    diff = background - Limg;
	    diff = diff > 15;
	    Limg.copyTo(_forground, diff);
	    forground = _forground;
	    imshow("Forground", forground);
	    }
	    else  cout << "Background is empty" << endl;
	    break;
    }
}

void DBcreator::mainLoop()
{    
    boost::thread getNextImages(boost::bind(&DBcreator::getNextImage,this));
    
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
            
            
            
            /*
  	        boost::thread L_RF(boost::bind(&RegionFinding::findRegions, &RegFind, _Limg));
            L_ROIs = RegFind.regions;
            L_ROIMasks = RegFind.masks;
            boost::thread R_RF(boost::bind(&RegionFinding::findRegions, &RegFind, _Rimg));
            R_ROIs = RegFind.regions;
            R_ROIMasks = RegFind.masks;
            L_RF.join();
            R_RF.join();
            */

		    	
            _Limg.copyTo(LimgDisp);
            _Rimg.copyTo(RimgDisp);
            
            displayImages();
            keyboardControl(); 
            tm.stop();
            //cout << "Full loop in " << tm.getTimeMilli() << " ms" << endl; 
            tm.reset();
        }
    }
}

//*/
int main(int argc, char** argv)
{
    DBcreator DB;
    DB.start();
}

//*/
